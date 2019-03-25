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

BLA_ret BLA(char *img, img w);

#ifdef __cplusplus
}
#endif

#endif