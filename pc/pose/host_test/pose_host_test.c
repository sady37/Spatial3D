/* Host harness: include the firmware translation unit to reach its statics,
 * feed gPoseIn from a file of 160 floats, print gPoseOut. */
#include <stdio.h>
#define main firmware_main_unused   /* pose_mlp.c has no main; guard anyway */
#include "pose_mlp.c"
#undef main
int main(int argc, char **argv)
{
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror("open"); return 1; }
    int nread = fread(gPoseIn, sizeof(float), POSE_INPUT_SIZE, f);
    fclose(f);
    if (nread != POSE_INPUT_SIZE) { fprintf(stderr, "short read %d\n", nread); return 1; }
    poseInfer();
    for (int c = 0; c < POSE_NUM_CLASSES; c++) printf("%.9e\n", gPoseOut[c]);
    return 0;
}
