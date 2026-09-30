// Records the TX path state every ~170 us in a ring and dumps the samples around the n-th underrun
// episode (first change of the gate's late/stale counters after a quiet second).
// Usage: episode [max_seconds] [which]
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include "liblitepcie.h"
#include "csr.h"
#define R 16384
#define PRE 600
#define POST 1500
static double mono(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
static uint64_t r64(int fd, uint32_t a){ uint64_t hi=litepcie_readl(fd,a), lo=litepcie_readl(fd,a+4); return (hi<<32)|lo; }
struct s { double t; int64_t rd; uint32_t late, stale, passed; int64_t armed_minus_tick; } buf[R];
int main(int argc,char**argv){
  double secs = argc>1?atof(argv[1]):40; int which = argc>2?atoi(argv[2]):1;
  int fd=open("/dev/m2sdr0",O_RDWR); if(fd<0){perror("open");return 1;}
  uint32_t c=litepcie_readl(fd,CSR_AD9361_TICK_CONTROL_ADDR);
  double t0=mono(), quiet_since=0; long n=0, ep=-1; int seen=0; uint32_t last=0;
  while(mono()-t0<secs){
    struct s *p=&buf[n%R];
    uint32_t l=litepcie_readl(fd,CSR_PCIE_DMA0_READER_TABLE_LOOP_STATUS_ADDR);
    p->t=mono()-t0; p->rd=(int64_t)(l>>16)*256+(l&0xffff);
    p->late=litepcie_readl(fd,CSR_TIMED_TX_LATE_COUNT_ADDR); p->stale=litepcie_readl(fd,CSR_TIMED_TX_STALE_COUNT_ADDR);
    p->passed=litepcie_readl(fd,CSR_TIMED_TX_PASSED_COUNT_ADDR);
    litepcie_writel(fd,CSR_AD9361_TICK_CONTROL_ADDR,c|(1u<<CSR_AD9361_TICK_CONTROL_READ_OFFSET));
    uint64_t tick=r64(fd,CSR_AD9361_TICK_READ_ADDR); uint64_t armed=r64(fd,CSR_TIMED_TX_ARMED_TS_ADDR);
    p->armed_minus_tick=(int64_t)(armed-tick);
    uint32_t cur=p->late+p->stale;
    if(n==0){ last=cur; quiet_since=p->t; }
    if(ep<0 && cur!=last){
      if(n>PRE && p->t-quiet_since>1.0 && ++seen==which) ep=n;
      quiet_since=p->t;
    }
    last=cur;
    if(ep>=0 && n>=ep+POST) break;
    n++; usleep(100);
  }
  if(ep<0){ printf("no episode in %.0f s (%ld samples, %d earlier events)\n",secs,n,seen); return 0; }
  long a=ep-PRE;
  double g=0, gt=0; int64_t mn=1LL<<60;
  for(long i=a+1;i<=ep;i++){ struct s *p=&buf[i%R],*q=&buf[(i-1)%R]; double d=p->t-q->t; if(d>g){g=d; gt=p->t-buf[ep%R].t;}
    if(p->armed_minus_tick>0 && p->armed_minus_tick<mn) mn=p->armed_minus_tick; }
  printf("episode at t=%.3f s. largest gap between probe samples in the %.0f ms before it: %.3f ms (at %.1f ms)\n",
         buf[ep%R].t,(buf[ep%R].t-buf[a%R].t)*1e3,g*1e3,gt*1e3);
  printf("columns: t_ms  dt_ms  reader_advance  late+ stale+ passed+  armed-tick(samples)\n");
  for(long i=a+1;i<=n && i<=ep+POST;i++){
    struct s *p=&buf[i%R],*q=&buf[(i-1)%R];
    double dt=(p->t-q->t)*1e3; long long adv=(p->rd-q->rd)&0xffffff;
    int interesting=(p->late!=q->late)||(p->stale!=q->stale)||(i>ep-4&&i<ep+6)||(i%100==0)||(adv>4)||(dt>0.5);
    if(!interesting) continue;
    if(i>ep+150 && (i%25)) continue;
    printf("%9.3f %6.3f  rd+%-4lld late+%-3u stale+%-4u passed+%-3u armed-tick=%lld\n",(p->t-buf[ep%R].t)*1e3,dt,adv,
      p->late-q->late,p->stale-q->stale,p->passed-q->passed,(long long)p->armed_minus_tick);
  }
  return 0;
}
