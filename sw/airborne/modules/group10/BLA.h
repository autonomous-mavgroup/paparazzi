//
// Created by matteo on 3/25/19.
//

#ifndef PAPARAZZI_BLA_H
#define PAPARAZZI_BLA_H

#ifdef  __cplusplus
extern "C" {
#endif

struct BLA_ret
{
    int heading;
    bool edge_reached;
};

struct BLA_ret BLA(char *img, int width, int height, float drone_height, float drone_theta);

#ifdef __cplusplus
}
#endif

#endif