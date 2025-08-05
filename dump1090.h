#ifndef DUMP1090_H
#define DUMP1090_H

#include <stdint.h>

struct modesMessage;
struct aircraft;
struct client;

int dump1090_run(int argc, char **argv);
void modesInitConfig(void);
void modesInit(void);
void modesInitRTLSDR(void);
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx);
void readDataFromFile(void);
void *readerThreadEntryPoint(void *arg);
void dumpMagnitudeBar(int index, int magnitude);
void dumpMagnitudeVector(uint16_t *m, uint32_t offset);
uint32_t modesChecksum(unsigned char *msg, int bits);
int modesMessageLenByType(int type);
int fixSingleBitErrors(unsigned char *msg, int bits);
int fixTwoBitsErrors(unsigned char *msg, int bits);
uint32_t ICAOCacheHashAddress(uint32_t a);
void addRecentlySeenICAOAddr(uint32_t addr);
int ICAOAddressWasRecentlySeen(uint32_t addr);
int bruteForceAP(unsigned char *msg, struct modesMessage *mm);
int decodeAC13Field(unsigned char *msg, int *unit);
int decodeAC12Field(unsigned char *msg, int *unit);
char *getMEDescription(int metype, int mesub);
void decodeModesMessage(struct modesMessage *mm, unsigned char *msg);
void displayModesMessage(struct modesMessage *mm);
void computeMagnitudeVector(void);
int detectOutOfPhase(uint16_t *m);
void applyPhaseCorrection(uint16_t *m);
void detectModeS(uint16_t *m, uint32_t mlen);
void useModesMessage(struct modesMessage *mm);
struct aircraft *interactiveCreateAircraft(uint32_t addr);
struct aircraft *interactiveFindAircraft(uint32_t addr);
int cprModFunction(int a, int b);
int cprNLFunction(double lat);
int cprNFunction(double lat, int isodd);
double cprDlonFunction(double lat, int isodd);
void decodeCPR(struct aircraft *a);
struct aircraft *interactiveReceiveData(struct modesMessage *mm);
void interactiveShowData(void);
void interactiveRemoveStaleAircrafts(void);
void snipMode(int level);
void modesInitNet(void);
void modesAcceptClients(void);
void modesFreeClient(int fd);
void modesSendAllClients(int service, void *msg, int len);
void modesSendRawOutput(struct modesMessage *mm);
void modesSendSBSOutput(struct modesMessage *mm, struct aircraft *a);
int hexDigitVal(int c);
int decodeHexMessage(struct client *c);
char *aircraftsToJson(int *len);
int handleHTTPRequest(struct client *c);
void modesReadFromClients(void);
void modesWaitReadableClients(int timeout_ms);
void sigWinchCallback(void);
int getTermRows(void);
void showHelp(void);
void backgroundTasks(void);

#endif /* DUMP1090_H */
