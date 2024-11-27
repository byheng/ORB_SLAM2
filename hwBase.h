#ifdef __cplusplus
extern "C"{
#endif

    int hwBaseInit(unsigned int dataInMode, unsigned int processModule);

    int hwBaseDeInit();

    void enableDisplay(char *IPStr);
    /*
    module:
    scaler  0
    ldc     1
    sgm     2

    bufferLeft/bufferRight:
    if you dont want,thisBuffer can be NULL
    yourself buffer to save data
    bufferLeft:leftSensor or sgmO
    bufferRight:RightSensor or sgmC

    len:
    buffer len

    flag:
    default 0：block to get frame
    */
    int getFrame(int module, char *bufferLeft, int len1, char *bufferRight, int len2, int flag);

#ifdef __cplusplus
}
#endif