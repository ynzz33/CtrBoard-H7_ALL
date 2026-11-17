#include "inference.hpp"
#include "model.hpp"
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <filesystem>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>
namespace jrl {
/* 使用系统sha256sum，exec直接传路径，不经过shell插值。 */
std::string sha256(const std::string& path) {
    const std::string resolved=std::filesystem::canonical(path).string();
    int fd[2];if(pipe(fd)) throw std::runtime_error("sha256 pipe");
    pid_t pid=fork();
    if(pid==0) { dup2(fd[1],STDOUT_FILENO);close(fd[0]);close(fd[1]);execlp("sha256sum","sha256sum","--",resolved.c_str(),nullptr);_exit(127); }
    close(fd[1]);if(pid<0) { close(fd[0]);throw std::runtime_error("sha256 fork"); }
    std::string output;char b[1024];ssize_t n;while((n=read(fd[0],b,sizeof(b)))>0) output.append(b,n);close(fd[0]);int status=0;waitpid(pid,&status,0);
    if(!WIFEXITED(status)||WEXITSTATUS(status)||output.size()<64) throw std::runtime_error("sha256sum failed: "+path);
    return output.substr(0,64);
}
struct TrtLog: nvinfer1::ILogger { void log(Severity s,const char* msg) noexcept override { if(s<=Severity::kWARNING) std::cerr<<"TensorRT: "<<msg<<'\n'; } };
static TrtLog log;
static void cuda_ok(cudaError_t e) { if(e!=cudaSuccess) throw std::runtime_error(cudaGetErrorString(e)); }
static std::vector<char> bytes(const std::string& p) { std::ifstream in(p,std::ios::binary);if(!in) throw std::runtime_error("cannot read "+p);return {std::istreambuf_iterator<char>(in),{}}; }
void Inference::convert(const std::string& onnx,const std::string& engine) {
    if(sha256(onnx)!=onnx_sha) throw std::runtime_error("ONNX is not the pinned policy4000PD200 export");
    if(std::filesystem::exists(engine)||std::filesystem::exists(engine+".meta")) throw std::runtime_error("refuse to overwrite engine");
    std::unique_ptr<nvinfer1::IBuilder> builder(nvinfer1::createInferBuilder(log));if(!builder) throw std::runtime_error("create builder");
    std::unique_ptr<nvinfer1::INetworkDefinition> net(builder->createNetworkV2(0));
    if(!net) throw std::runtime_error("create network");
    std::unique_ptr<nvonnxparser::IParser> parser(nvonnxparser::createParser(*net,log));
    if(!parser) throw std::runtime_error("create ONNX parser");
    if(!parser->parseFromFile(onnx.c_str(),int(nvinfer1::ILogger::Severity::kWARNING))) throw std::runtime_error("ONNX parse failed");
    if(net->getNbInputs()!=2||net->getNbOutputs()!=1) throw std::runtime_error("wrong model interface");
    std::unique_ptr<nvinfer1::IBuilderConfig> config(builder->createBuilderConfig());
    if(!config) throw std::runtime_error("create builder config");
    config->clearFlag(nvinfer1::BuilderFlag::kTF32); // FP32基线，不启用FP16/INT8。
    config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE,64u*1024u*1024u);
    std::unique_ptr<nvinfer1::IHostMemory> blob(builder->buildSerializedNetwork(*net,*config));if(!blob) throw std::runtime_error("build engine failed");
    std::ofstream out(engine,std::ios::binary);out.write(static_cast<const char*>(blob->data()),blob->size());out.close();if(!out) throw std::runtime_error("write engine failed");
    std::ofstream meta(engine+".meta");meta<<"checkpoint_sha256="<<checkpoint_sha<<"\ncontract="<<JRL_CONTRACT<<"\nonnx_sha256="<<sha256(onnx)<<"\nengine_sha256="<<sha256(engine)<<"\nprecision=FP32_no_TF32\n";
    meta.close();if(!meta) throw std::runtime_error("write metadata failed");
}
struct Inference::Impl {
    std::unique_ptr<nvinfer1::IRuntime> runtime;
    std::unique_ptr<nvinfer1::ICudaEngine> engine;
    std::unique_ptr<nvinfer1::IExecutionContext> context;
    cudaStream_t stream{};void* device[3]{};
    ~Impl() { if(stream) cudaStreamSynchronize(stream);for(auto p:device) if(p) cudaFree(p);if(stream) cudaStreamDestroy(stream); }
};
Inference::Inference(const std::string& path):impl_(new Impl) {
    std::ifstream meta(path+".meta");std::string text((std::istreambuf_iterator<char>(meta)),{});
    if(text.find(std::string("onnx_sha256=")+onnx_sha+"\n")==std::string::npos || text.find("contract="+std::to_string(JRL_CONTRACT)+"\n")==std::string::npos || text.find("engine_sha256="+sha256(path)+"\n")==std::string::npos) throw std::runtime_error("engine metadata/hash mismatch");
    auto data=bytes(path);auto& x=*impl_;
    x.runtime.reset(nvinfer1::createInferRuntime(log));if(!x.runtime) throw std::runtime_error("create runtime");
    x.engine.reset(x.runtime->deserializeCudaEngine(data.data(),data.size()));if(!x.engine) throw std::runtime_error("load engine failed");
    if(x.engine->getNbIOTensors()!=3) throw std::runtime_error("wrong tensor count");
    x.context.reset(x.engine->createExecutionContext());if(!x.context) throw std::runtime_error("create context");
    cuda_ok(cudaStreamCreate(&x.stream));
    const char* names[]={"obs","obs_history","actions"};const int sizes[]={25,125,6};
    for(unsigned i=0;i<3;i++) {
        auto d=x.engine->getTensorShape(names[i]);
        if(d.nbDims!=2||d.d[0]!=1||d.d[1]!=sizes[i]||x.engine->getTensorDataType(names[i])!=nvinfer1::DataType::kFLOAT ||
           x.engine->getTensorIOMode(names[i])!=(i==2 ? nvinfer1::TensorIOMode::kOUTPUT:nvinfer1::TensorIOMode::kINPUT)) throw std::runtime_error("wrong tensor shape/type/name");
        cuda_ok(cudaMalloc(&x.device[i],sizes[i]*sizeof(float)));
        if(!x.context->setTensorAddress(names[i],x.device[i])) throw std::runtime_error("bind tensor failed");
    }
    Input zero{};for(int i=0;i<20;i++) (void)run(zero); // 加载/预热不进入实时闭环。
}
Inference::~Inference()=default;
Action Inference::run(const Input& input) {
    for(float f:input) if(!std::isfinite(f)) throw std::runtime_error("invalid input");
    auto& x=*impl_;Action result;
    cuda_ok(cudaMemcpyAsync(x.device[0],input.data(),25*4,cudaMemcpyHostToDevice,x.stream));
    cuda_ok(cudaMemcpyAsync(x.device[1],input.data()+25,125*4,cudaMemcpyHostToDevice,x.stream));
    if(!x.context->enqueueV3(x.stream)) throw std::runtime_error("inference failed");
    cuda_ok(cudaMemcpyAsync(result.data(),x.device[2],6*4,cudaMemcpyDeviceToHost,x.stream));cuda_ok(cudaStreamSynchronize(x.stream));
    for(float f:result) { if(!std::isfinite(f)) throw std::runtime_error("invalid output"); }
    return result;
}
}
