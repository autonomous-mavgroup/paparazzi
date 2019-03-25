

#ifndef LINE_DETECTOR_OPENCV_H
#define LINE_DETECTOR_OPENCV_H
#include "modules/group10/setting.h"

#ifdef __cplusplus
extern "C" {
#endif

int detect_line_opencv(char *img, int width, int height, char *out, struct settings set_conf);

#ifdef __cplusplus
}
#endif

#endif
