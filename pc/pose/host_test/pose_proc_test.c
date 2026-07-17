/* Drive PoseMlp_process with a scripted sequence read from a text file:
 *   line 1: zOffset
 *   then repeated frames:
 *     "F <numTargets> <numPoints>"
 *     numTargets lines: tid posX posY posZ velY velZ accY accZ
 *     numPoints  lines: x y z snr
 * After each frame, prints: tid pose fallingProb valid  (one line per result)
 * separated by "---" per frame. */
#include <stdio.h>
#include <stdlib.h>
#define main firmware_main_unused
#include "pose_mlp.c"
#undef main
int main(int argc, char **argv){
    FILE*f=fopen(argv[1],"r"); if(!f){perror("open");return 1;}
    float zoff; if(fscanf(f,"%f",&zoff)!=1)return 1;
    PoseMlp_init(); PoseMlp_setZOffset(zoff);
    char tag[8];
    while(fscanf(f,"%7s",tag)==1){
        int nt,np; if(fscanf(f,"%d %d",&nt,&np)!=2)break;
        PoseTrackKin kin[16]; PosePoint pts[256]; PoseResult out[16];
        for(int i=0;i<nt;i++){ unsigned tid; float a,b,c,d,e,g,h;
            fscanf(f,"%u %f %f %f %f %f %f %f",&tid,&a,&b,&c,&d,&e,&g,&h);
            kin[i].tid=tid; kin[i].posX=a; kin[i].posY=b; kin[i].posZ=c;
            kin[i].velY=d; kin[i].velZ=e; kin[i].accY=g; kin[i].accZ=h; }
        for(int i=0;i<np;i++){ float x,y,z,s; fscanf(f,"%f %f %f %f",&x,&y,&z,&s);
            pts[i].x=x; pts[i].y=y; pts[i].z=z; pts[i].snr=s; }
        uint32_t w=PoseMlp_process(kin,nt,pts,np,out);
        for(uint32_t i=0;i<w;i++)
            printf("%u %u %u %u\n",out[i].tid,out[i].pose,out[i].fallingProb,out[i].valid);
        printf("---\n");
    }
    return 0;
}
