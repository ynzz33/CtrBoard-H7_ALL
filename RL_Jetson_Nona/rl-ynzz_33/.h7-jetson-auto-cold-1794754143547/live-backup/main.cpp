#include <cstdlib>
#include <random>
#include "jetson_shadow_protocol.h"
#include <cmath>
#include <cstring>
#include "inference.hpp"
#include "logger.hpp"
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <map>
#include <poll.h>
#include <random>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
namespace {
volatile std::sig_atomic_t stopping=0;
void stop_signal(int) { stopping=1; }
struct Serial {
    int fd=-1;
    explicit Serial(const std::string& path) {
        fd=open(path.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);if(fd<0) throw std::runtime_error("cannot open serial: "+path);
        if(ioctl(fd,TIOCEXCL)) { close(fd);fd=-1;throw std::runtime_error("serial already owned"); }
        termios t{};
        if(tcgetattr(fd,&t)) { close(fd);fd=-1;throw std::runtime_error("tcgetattr"); }
        cfmakeraw(&t);cfsetispeed(&t,B1152000);cfsetospeed(&t,B1152000);
        t.c_cflag=(t.c_cflag&~(PARENB|CSTOPB|CSIZE|CRTSCTS))|CS8|CLOCAL|CREAD;
        if(tcsetattr(fd,TCSANOW,&t)) { close(fd);fd=-1;throw std::runtime_error("tcsetattr"); }
        tcflush(fd,TCIFLUSH);
    }
    ~Serial() { if(fd>=0) close(fd); }
    void send(const uint8_t* p,size_t n) {
        uint64_t end=jrl::monotonic_us()+2000u;size_t sent=0;
        while(sent<n) {
            ssize_t k=write(fd,p+sent,n-sent);
            if(k>0) sent+=size_t(k);
            else if(k<0&&errno!=EAGAIN&&errno!=EINTR) throw std::runtime_error("serial write failed");
            if(sent<n) { if(jrl::monotonic_us()>=end) throw std::runtime_error("serial write deadline");pollfd d{fd,POLLOUT,0};poll(&d,1,1); }
        }
    }
};
using Args=std::map<std::string,std::string>;
std::string need(const Args& a,const char* key) { auto it=a.find(key);if(it==a.end()) throw std::runtime_error(std::string("missing ")+key);return it->second; }
void vectors(jrl::Inference& net,const std::string& input,const std::string& output) {
    if(std::filesystem::exists(output)) throw std::runtime_error("refuse to overwrite output");
    std::ifstream in(input,std::ios::binary);std::ofstream out(output,std::ios::binary);if(!in||!out) throw std::runtime_error("vector files");
    jrl::Input x;size_t count=0;uint64_t start=jrl::monotonic_us();std::vector<uint64_t> timings;
    while(in.read(reinterpret_cast<char*>(x.data()),sizeof(x))) { auto begin=jrl::monotonic_us();auto y=net.run(x);timings.push_back(jrl::monotonic_us()-begin);out.write(reinterpret_cast<char*>(y.data()),sizeof(y));count++; }
    if(in.gcount()!=0||!in.eof()||!out||!count) throw std::runtime_error("incomplete vectors");
    std::sort(timings.begin(),timings.end());
    std::cout<<"vectors="<<count<<" total_us="<<jrl::monotonic_us()-start<<" inference_p95_us="<<timings[(timings.size()-1)*95/100]<<" inference_max_us="<<timings.back()<<'\n';
}
void replay(jrl::Inference& net,const std::string& input,const std::string& output) {
    if(std::filesystem::exists(output)) throw std::runtime_error("refuse to overwrite replay");
    std::ifstream in(input,std::ios::binary);std::ofstream out(output);if(!in||!out) throw std::runtime_error("replay files");
    out<<"sequence,max_abs_error,match\n";size_t count=0;bool all=true;uint8_t h[16];
    while(in.read(reinterpret_cast<char*>(h),16)) {
        uint32_t n=jrl_u32(h+4);if(n>4096) throw std::runtime_error("bad log record size");std::vector<uint8_t> b(n);
        if(!in.read(reinterpret_cast<char*>(b.data()),n)) throw std::runtime_error("truncated log payload");
        if(jrl_u32(h)!=3) continue;
        if(n!=640) throw std::runtime_error("bad inference record");
        jrl::Input x;for(unsigned i=0;i<150;i++) x[i]=jrl_f32(b.data()+16+4*i);
        jrl::Action y{};if(!jrl_u32(b.data()+12)) y=net.run(x);
        bool ok=true;float error=0;
        for(unsigned i=0;i<6;i++) { float actual=jrl::clip(y[i],100.f),expected=jrl_f32(b.data()+616+4*i),e=std::abs(actual-expected);error=std::max(error,e);ok&=e<=1e-4f+1e-4f*std::abs(expected); }
        out<<jrl_u32(b.data()+4)<<','<<error<<','<<ok<<'\n';all&=ok;count++;
    }
    if(in.gcount()!=0||!in.eof()||!out||!count||!all) throw std::runtime_error("replay incomplete or mismatch");
}
void shadow(const Args& a) {
    jrl::Inference net(need(a,"--engine"));
    const auto seconds=std::stoull(need(a,"--seconds"));
    if (seconds>86400) throw std::runtime_error("invalid duration");
    jrl::Logger log(need(a,"--log-dir"));
    Serial serial(need(a,"--port"));
    std::signal(SIGINT,stop_signal);
    std::signal(SIGTERM,stop_signal);
    jrl_parser_t parser{};
    jrl::Input input{};
    uint8_t identity[24]{};
    bool first=false;
    unsigned replies=0;
    const auto deadline=jrl::monotonic_us()+seconds*1000000u;
    const char* mode = std::getenv("JRL_EXTERNAL_ACTION");
    const bool external = mode && std::strcmp(mode, "1") == 0;
    const uint32_t contract = external ? JRL_EXTERNAL_CONTRACT : JRL_SHADOW_CONTRACT;
    std::random_device random;
    const uint64_t session = external ? ((uint64_t(random()) << 32) | random() | 1u) : 0u;
    uint8_t ready[JRL_HEADER + JRL_SHADOW_READY_BYTES + 4]{};
    jrl_header(ready, JRL_SHADOW_READY, JRL_SHADOW_READY_BYTES, session, 0, 0, 0);
    jrl_p32(ready + JRL_HEADER, contract);
    const auto ready_length = jrl_finish(ready);
    uint64_t next_ready_us = 0;
    while (!stopping && (!seconds || jrl::monotonic_us()<deadline)) {
        const auto now = jrl::monotonic_us();
        if (now >= next_ready_us) {
            serial.send(ready, ready_length);
            log.push(2, ready, ready_length);
            next_ready_us = now + 50000u;
        }
        pollfd fd{serial.fd,POLLIN,0};
        const int rc=poll(&fd,1,20);
        if (rc<0 && errno==EINTR) continue;
        if (rc<0 || fd.revents & (POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("USB disconnected");
        if (!(fd.revents & POLLIN)) continue;
        uint8_t data[4096];
        const auto n=read(serial.fd,data,sizeof(data));
        if (n<0 && (errno==EINTR || errno==EAGAIN)) continue;
        if (n<=0) throw std::runtime_error("USB read failed");
        log.push(1,data,size_t(n));
        for (ssize_t i=0;i<n;++i) {
            if (!jrl_feed(&parser,data[i])) continue;
            const auto* msg=parser.data;
            const auto type=msg[5];
            if (type!=JRL_SHADOW_INPUT_FIRST && type!=JRL_SHADOW_INPUT_SECOND) continue;
            if (jrl_u16(msg+6)!=JRL_SHADOW_INPUT_BYTES || jrl_u32(msg+32)!=contract || jrl_u64(msg+8)!=session) {
                first=false;
                continue;
            }
            const bool beginning=type==JRL_SHADOW_INPUT_FIRST;
            if (beginning) {
                std::memcpy(identity,msg+8,sizeof(identity));
                first=true;
            } else if (!first || std::memcmp(identity,msg+8,sizeof(identity))) {
                first=false;
                continue;
            }
            const unsigned offset=beginning?0:JRL_SHADOW_HALF_FLOATS;
            for (unsigned j=0;j<JRL_SHADOW_HALF_FLOATS;++j) input[offset+j]=jrl_f32(msg+36+4*j);
            if (beginning) continue;
            first=false;
            for (float v:input) if (!std::isfinite(v)) throw std::runtime_error("nonfinite input");
            const auto start=jrl::monotonic_us();
            const auto action=net.run(input);
            const auto us=jrl::monotonic_us()-start;
            uint8_t reply[JRL_HEADER+JRL_SHADOW_REPLY_BYTES+4]{};
            jrl_header(reply,JRL_SHADOW_REPLY,JRL_SHADOW_REPLY_BYTES,0,0,0,0);
            std::memcpy(reply+8,identity,sizeof(identity));
            jrl_p32(reply+32,contract);
            for (unsigned j=0;j<action.size();++j) {
                if (!std::isfinite(action[j])) throw std::runtime_error("nonfinite action");
                jrl_pf32(reply+36+4*j,action[j]);
            }
            jrl_p32(reply+60,uint32_t(us));
            const auto length=jrl_finish(reply);
            serial.send(reply,length);
            log.push(2,reply,length);
            ++replies;
        }
    }
    std::cout<<"shadow_replies="<<replies<<" parser_errors="<<parser.errors<<'\n';
    if (!replies && seconds && !stopping) throw std::runtime_error("no shadow inputs received");
}
}
int main(int argc,char** argv) {
    try {
        if(argc<2) throw std::runtime_error("commands: convert, vectors, replay, shadow");
        Args a;for(int i=2;i<argc;i+=2) { if(i+1>=argc) throw std::runtime_error("options require values");a[argv[i]]=argv[i+1]; }
        std::string mode=argv[1];
        if(mode=="convert") jrl::Inference::convert(need(a,"--onnx"),need(a,"--engine"));
        else if(mode=="vectors"||mode=="replay") { jrl::Inference net(need(a,"--engine"));if(mode=="vectors") vectors(net,need(a,"--input"),need(a,"--output"));else replay(net,need(a,"--input"),need(a,"--output")); }
        
        else if(mode=="shadow") shadow(a);
        else throw std::runtime_error("unknown command");
        return 0;
    } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 1; }
}
