#include "digipasDWL.hpp"

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>

static void poll_thread_C(void* pPvt) {
    DigipasDWL* pDigipasDWL = (DigipasDWL*)pPvt;
    pDigipasDWL->poll();
}

constexpr int MAX_CONTROLLERS = 1;

DigipasDWL::DigipasDWL(const char* asyn_port)
    : asynPortDriver((std::string(asyn_port) + "_internal").c_str(), MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0) {

    asynStatus status = pasynOctetSyncIO->connect(asyn_port, 0, &pasynUserDriver_, NULL);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Failed to connect to sensor\n");
        return;
    }

    std::cout << "initializing sensor..." << std::endl;
    init_sensor();
    epicsThreadSleep(1.0);

    std::cout << "Setting location..." << std::endl;
    set_location(0x42, 0x05); // United States, Chicago
    epicsThreadSleep(1.0);

    std::cout << "Setting sensor to Dual Mode..." << std::endl;
    set_mode(SensorMode::Dual);
    epicsThreadSleep(1.0);

    std::cout << "Starting polling loop..." << std::endl;
    epicsThreadCreate("DigipasDWLPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void DigipasDWL::poll() {
    while (true) {
        lock();

        // do stuff ...
        std::cout << "Running... " << count_ << std::endl;
        count_ += 1;

        // poll sensor
	read_sensor();

        callParamCallbacks();
        unlock();
        epicsThreadSleep(1.0);
    }
}
//
// asynStatus (*write)(asynUser *pasynUser,
// char const *buffer, size_t out_buffer_len,
// double timeout,size_t *nbytesTransfered);
// asynStatus (*read)(asynUser *pasynUser, char *buffer, size_t buffer_len,
		  // double timeout, size_t *nbytesTransfered,int *eomReason);

void DigipasDWL::init_sensor() {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x24;
    size_t ntransfered;
    asynStatus status = pasynOctetSyncIO->write(pasynUserDriver_, out_buffer_.data(), out_buffer_.size(), IO_TIMEOUT, &ntransfered);
    if (status) {
	asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "write failed in function init_sensor()\n");
	return;
    }
}

void DigipasDWL::set_location(char country, char city) {
    // 0x06, 0x01, 0x08, country, city, 0x00, 0x00, 0x5A, 0x00, 0x00, 0x00, 0x00
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = static_cast<char>(SensorMode::Location);
    out_buffer_[3] = country;
    out_buffer_[4] = city;
    out_buffer_[5] = 0x00;
    out_buffer_[6] = 0x00;
    out_buffer_[7] = 0x5A;
    size_t ntransfered;
    asynStatus status = pasynOctetSyncIO->write(pasynUserDriver_, out_buffer_.data(), out_buffer_.size(), IO_TIMEOUT, &ntransfered);
    if (status) {
	asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "write failed in function set_location()\n");
	return;
    }
}

double get_x(const std::array<char, BUFFER_SIZE>& data) {
    double x = (((data[7] << 16) + (data[6] << 8) + data[5]) - 3000000) / 100000.0;
    return x;
}

double get_y(const std::array<char, BUFFER_SIZE>& data) {
    double y = (((data[4] << 16) + (data[3] << 8) + data[2]) - 3000000) / 100000.0;
    return y;
}

void DigipasDWL::read_sensor() {
    in_buffer_.fill(0x0);
    size_t ntransfered;
    int eom_reason;
    asynStatus status = pasynOctetSyncIO->read(pasynUserDriver_, in_buffer_.data(), in_buffer_.size(), IO_TIMEOUT, &ntransfered, &eom_reason);
    if (status) {
	asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "read_sensor() failed[%d]. Bytes transfered: %ld\n", eom_reason, ntransfered);
	return;
    }

    std::cout << "bytes transferred: " << ntransfered << std::endl;

    auto x = get_x(in_buffer_);
    std::cout << "x = " << x << "deg" << std::endl;

    auto y = get_y(in_buffer_);
    std::cout << "y = " << y << "deg" << std::endl;
}

void DigipasDWL::set_mode(SensorMode mode) {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = static_cast<char>(mode);
    out_buffer_[3] = 0xAA;
    size_t nout;
    size_t nin;
    int eom_reason;
    asynStatus status = pasynOctetSyncIO->writeRead(pasynUserDriver_, out_buffer_.data(), out_buffer_.size(),
	    in_buffer_.data(), in_buffer_.size(), IO_TIMEOUT, &nout, &nin, &eom_reason);
    if (status) {
	asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "set_mode() failed[%d]\n", eom_reason);
    }
}

asynStatus DigipasDWL::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    // int function = pasynUser->reason;
    bool comm_ok = true;

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

asynStatus DigipasDWL::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {
    // int function = pasynUser->reason;
    bool comm_ok = true;

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

// register function for iocsh
extern "C" int DigipasDWLConfig(const char* asyn_port_name) {
    DigipasDWL* pDigipasDWL = new DigipasDWL(asyn_port_name);
    pDigipasDWL = NULL;
    return (asynSuccess);
}

static const iocshArg DigipasDWLArg0 = {"asyn port name", iocshArgString};
static const iocshArg* const DigipasDWLArgs[1] = {&DigipasDWLArg0};
static const iocshFuncDef DigipasDWLFuncDef = {"DigipasDWLConfig", 1, DigipasDWLArgs};

static void DigipasDWLCallFunc(const iocshArgBuf* args) { DigipasDWLConfig(args[0].sval); }

void DigipasDWLRegister(void) { iocshRegister(&DigipasDWLFuncDef, DigipasDWLCallFunc); }

extern "C" {
epicsExportRegistrar(DigipasDWLRegister);
}
