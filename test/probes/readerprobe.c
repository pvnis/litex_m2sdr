// Samples the TX DMA reader's table index at ~10 kHz and reports how fast it advances.
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include "liblitepcie.h"
#include "csr.h"
static double mono(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
int main(int argc,char**argv){
  double secs = argc>1?atof(argv[1]):20;
  int fd=open("/dev/m2sdr0",O_RDWR); if(fd<0){perror("open");return 1;}
  uint32_t l=litepcie_readl(fd,CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_ADDR);
  int64_t prev=(int64_t)(l>>16)*256+(l&0xffff); double t0=mono(), tp=t0;
  long n=0, bursts=0; double maxrate=0; int64_t total=0, burst_frames=0;
  while(mono()-t0<secs){
    usleep(100);
    l=litepcie_readl(fd,CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_ADDR); double t=mono();
    int64_t cur=(int64_t)(l>>16)*256+(l&0xffff); int64_t d=(cur-prev)&0xffffff; if(d>0x800000) d=0;
    double rate=d/(t-tp); total+=d; n++;
    if(rate>maxrate) maxrate=rate;
    if(d>=6 && rate>30000){ bursts++; burst_frames+=d; }   // nominal is 11272 frames/s
    prev=cur; tp=t;
  }
  printf("samples=%ld avg=%.0f frames/s, max instantaneous=%.0f frames/s, intervals above 30k frames/s: %ld (%lld frames)\n",
         n,total/(mono()-t0),maxrate,bursts,(long long)burst_frames);
  return 0;
}
