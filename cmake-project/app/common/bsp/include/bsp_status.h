#ifndef BSP_STATUS_H
#define BSP_STATUS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_OK =0,
    BSP_ERROR,
    BSP_BUSY,
    BSP_TIMEOUT,
    BSP_INVALID_ARGUMENT,
}bsp_status_t;
    
#ifdef __cplusplus
}
#endif

#endif/*BSP_STATUS_H*/
