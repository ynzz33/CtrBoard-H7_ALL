/* 完整测量传输：固定使用板载USB CDC；14B H7TC只启停记录，不访问控制。
 * 数据仍为参考H7OUT001/H7LAT001；追加34B状态包H7STAT01用于会话头尾确认。
 * 协议均小端，CRC16/CCITT-FALSE；USB忙/断连不复用异步缓冲。
 */
#include "machine_config.h"
#include "latency_usb.h"
#if MACHINE_LATENCY_CAPTURE_ENABLE
#include "latency_capture.h"
#include "output_capture.h"
#include "usbd_cdc_if.h"
#include "mono_ns.h"
#include <string.h>
#include "dma_cache.h"
static uint8_t tx[OUTPUT_CAPTURE_PACKET_MAX+LATENCY_PACKET_MAX+34u] __attribute__((aligned(DMA_CACHE_LINE_SIZE)));
static uint8_t command[14],command_size,state;
static volatile uint8_t request;
static volatile uint32_t requested_token;
static uint32_t token;
static uint16_t crc16(const uint8_t *p,uint16_t n)
{
    uint16_t crc=0xffffu;
    for(uint16_t i=0u;i<n;i++) { crc^=(uint16_t)p[i]<<8;for(uint8_t b=0u;b<8u;b++) { crc=crc&0x8000u?(uint16_t)((crc<<1)^0x1021u):(uint16_t)(crc<<1); } }
    return crc;
}
void Latency_Usb_Receive(const uint8_t *data,uint32_t size)
{
    for(uint32_t i=0u;i<size;i++)
    {
        command[command_size++]=data[i];
        while(command_size>=4u && memcmp(command,"H7TC",4u)) { memmove(command,command+1u,--command_size); }
        if(command_size==14u)
        {
            uint16_t expected;memcpy(&expected,command+12u,2u);
            if(command[5]==1u && command[6]==0u && command[7]==0u && (command[4]==1u || command[4]==2u) && expected==crc16(command,12u))
            { memcpy((void*)&requested_token,command+8u,4u);__DMB();request=command[4];command_size=0u; }
            else { memmove(command,command+1u,--command_size); }
        }
    }
}
void Latency_Usb_Pump(void)
{
    uint16_t first,second,length,crc,count_a,count_b;
    uint32_t queued_a,queued_b,pending,mask;
    uint8_t op,next_state;uint32_t next_token;
    uint64_t stamp;
    mask=__get_PRIMASK();__disable_irq();op=request;next_token=requested_token;request=0u;__set_PRIMASK(mask);
    if(op==1u && (state==0u || state==3u)) { token=next_token;latency_recording=1u;state=1u; }
    if(op==2u && token==next_token && state==1u) { latency_recording=0u;state=2u; }
    if(!CDC_Transmit_Ready_HS()) { return; }
    first=Output_Capture_Packet(tx);second=Latency_Capture_Packet(tx+first);length=first+second;
    memcpy(&queued_a,tx+28u,4u);memcpy(&count_a,tx+12u,2u);
    memcpy(&queued_b,tx+first+28u,4u);memcpy(&count_b,tx+first+12u,2u);memcpy(&pending,tx+first+36u,4u);
    next_state=state;
    if(state==2u && queued_a==count_a && queued_b==count_b && pending==0u) { next_state=3u; }
    memcpy(tx+length,"H7STAT01",8u);tx[length+8u]=1u;tx[length+9u]=next_state;memset(tx+length+10u,0u,2u);
    memcpy(tx+length+12u,&token,4u);queued_a-=count_a;queued_b-=count_b;
    memcpy(tx+length+16u,&queued_a,4u);memcpy(tx+length+20u,&queued_b,4u);memcpy(tx+length+24u,&pending,4u);
    stamp=Mono_Ns_Get(); /* 状态末字段为板端低32位微秒，绝不与主机时间相减。 */
    { uint32_t us=(uint32_t)(stamp/1000u);memcpy(tx+length+28u,&us,4u); }
    crc=crc16(tx+length,32u);memcpy(tx+length+32u,&crc,2u);length+=34u;
    if(CDC_Transmit_HS(tx,length)!=USBD_OK) { return; }
    Output_Capture_Commit();Latency_Capture_Commit();state=next_state;
}
#else
void Latency_Usb_Pump(void) {}
void Latency_Usb_Receive(const uint8_t *data,uint32_t size) { (void)data;(void)size; }
#endif
