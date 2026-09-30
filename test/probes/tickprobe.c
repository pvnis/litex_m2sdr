// Sample-counter probe: tick rate, time-base mode, gate/fine-gate counters.
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include "liblitepcie.h"
#include "csr.h"
static uint64_t r64(int fd, uint32_t a){ uint64_t hi=litepcie_readl(fd,a), lo=litepcie_readl(fd,a+4); return (hi<<32)|lo; }
static uint64_t tick_now(int fd){
  uint32_t c=litepcie_readl(fd,CSR_AD9361_TICK_CONTROL_ADDR);
  litepcie_writel(fd,CSR_AD9361_TICK_CONTROL_ADDR,c|(1u<<CSR_AD9361_TICK_CONTROL_READ_OFFSET));
  return r64(fd,CSR_AD9361_TICK_READ_ADDR);
}
static double mono(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
int main(int argc,char**argv){
  int n = argc>1?atoi(argv[1]):3; int us = argc>2?atoi(argv[2]):1000000;
  int fd=open("/dev/m2sdr0",O_RDWR); if(fd<0){perror("open");return 1;}
  uint64_t pt=0; double pw=0; uint32_t ph=0,pl=0,pp=0,ps=0;
  for(int i=0;i<n;i++){
    uint64_t t=tick_now(fd); double w=mono();
    uint32_t ctrl=litepcie_readl(fd,CSR_AD9361_TICK_CONTROL_ADDR), st=litepcie_readl(fd,CSR_AD9361_TICK_STATUS_ADDR);
    uint32_t held=litepcie_readl(fd,CSR_TIMED_TX_HELD_COUNT_ADDR), late=litepcie_readl(fd,CSR_TIMED_TX_LATE_COUNT_ADDR);
    uint32_t stale=litepcie_readl(fd,CSR_TIMED_TX_STALE_COUNT_ADDR), passed=litepcie_readl(fd,CSR_TIMED_TX_PASSED_COUNT_ADDR);
    uint64_t armed=r64(fd,CSR_TIMED_TX_ARMED_TS_ADDR);
    printf("tick=%llu (%s) mode{ticks=%u fine=%u} rx_ovf=%u tx_wait=%u trimmed=%u | gate: armed-tick=%lld held=%u late=%u stale=%u passed=%u",
      (unsigned long long)t,(t&1)?"odd":"even",ctrl&1,(ctrl>>3)&1,st&1,(st>>1)&1,(st>>16)&0xffff,(long long)(armed-t),held,late,stale,passed);
    if(pw>0){ double dt=w-pw; printf(" | rates/s: tick %.0f held %.0f late %.0f stale %.0f passed %.0f",(double)(t-pt)/dt,(held-ph)/dt,(late-pl)/dt,(stale-ps)/dt,(passed-pp)/dt); }
    printf("\n"); pt=t; pw=w; ph=held; pl=late; ps=stale; pp=passed;
    if(i+1<n) usleep(us);
  }
  return 0;
}
