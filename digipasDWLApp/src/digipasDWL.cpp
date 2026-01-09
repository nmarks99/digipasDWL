#include "digipasDWL.hpp"

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <algorithm>

static void poll_thread_C(void* pPvt) {
    DigipasDWL* pDigipasDWL = (DigipasDWL*)pPvt;
    pDigipasDWL->poll();
}

constexpr int MAX_CONTROLLERS = 1;
constexpr int INTERFACE_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask;
constexpr int INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask;
constexpr int ASYN_FLAGS = ASYN_MULTIDEVICE | ASYN_CANBLOCK;

DigipasDWL::DigipasDWL(const char* conn_port, const char* driver_port, std::string mode_str, int country, int city)
    : asynPortDriver(driver_port, MAX_CONTROLLERS, INTERFACE_MASK, INTERRUPT_MASK, ASYN_FLAGS, 1, 0, 0) {

    asynStatus status = pasynOctetSyncIO->connect(conn_port, 0, &pasynUserDriver_, NULL);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Failed to connect to sensor\n");
        return;
    }

    createParam(X_DEG_STRING, asynParamFloat64, &xdegId_);
    createParam(Y_DEG_STRING, asynParamFloat64, &ydegId_);

    status = init_sensor();
    if (status)
        return;

    status = set_location(country, city);
    if (status)
        return;

    std::transform(mode_str.begin(), mode_str.end(), mode_str.begin(), [](auto& c){
	return std::tolower(c);
    });

    SensorMode sensor_mode = SensorMode::None;
    if (mode_str == "single") {
	sensor_mode = SensorMode::Single;
    } else if (mode_str == "dual") {
	sensor_mode = SensorMode::Dual;
    } else if (mode_str == "calibration") {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "Calibration mode not supported\n");
        return;
    }

    status = set_mode(sensor_mode);
    if (status)
        return;

    epicsThreadCreate("DigipasDWLPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void DigipasDWL::poll() {
    while (true) {
        lock();

        get_angles();

        callParamCallbacks();
        unlock();
        epicsThreadSleep(1.0);
    }
}

asynStatus DigipasDWL::init_sensor() {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x24;
    return write_read();
}

asynStatus DigipasDWL::set_location(char country, char city) {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = static_cast<char>(SensorMode::Location);
    out_buffer_[3] = country;
    out_buffer_[4] = city;
    out_buffer_[5] = 0x00;
    out_buffer_[6] = 0x00;
    out_buffer_[7] = 0x5A;
    return write_read();
}

double compute_x(const std::array<char, BUFFER_SIZE>& data) {
    return (((data[7] << 16) + (data[6] << 8) + data[5]) - 3000000) / 100000.0;
}

double compute_y(const std::array<char, BUFFER_SIZE>& data) {
    return (((data[4] << 16) + (data[3] << 8) + data[2]) - 3000000) / 100000.0;
}

asynStatus DigipasDWL::get_angles() {
    asynStatus status = read();
    if (status)
        return status;

    double x_deg = compute_x(in_buffer_);
    double y_deg = compute_y(in_buffer_);

    setDoubleParam(xdegId_, x_deg);
    setDoubleParam(ydegId_, y_deg);

    return status;
}

asynStatus DigipasDWL::set_mode(SensorMode mode) {
    out_buffer_.fill(0x0);
    out_buffer_[0] = 0x06;
    out_buffer_[1] = 0x01;
    out_buffer_[2] = static_cast<char>(mode);
    out_buffer_[3] = 0xAA;

    asynStatus status = write_read();
    if (status)
        return status;

    return status;

    mode_ = mode;
}

asynStatus DigipasDWL::read() {
    size_t nbytesin;
    int eom_reason;
    asynStatus status = pasynOctetSyncIO->read(pasynUserDriver_, in_buffer_.data(), in_buffer_.size(),
                                               IO_TIMEOUT, &nbytesin, &eom_reason);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR, "pasynOctetSyncIO->read() failed[%d]. Read %ld bytes\n",
                  eom_reason, nbytesin);
    }
    return status;
}

asynStatus DigipasDWL::write_read() {
    size_t nbytesin;
    size_t nbytesout;
    int eom_reason;
    asynStatus status = pasynOctetSyncIO->writeRead(pasynUserDriver_, out_buffer_.data(), out_buffer_.size(),
                                                    in_buffer_.data(), in_buffer_.size(), IO_TIMEOUT,
                                                    &nbytesout, &nbytesin, &eom_reason);
    if (status) {
        asynPrint(pasynUserDriver_, ASYN_TRACE_ERROR,
                  "pasynOctetSyncIO->writeRead() failed[%d]. Wrote %ld, read %ld bytes\n", eom_reason,
                  nbytesout, nbytesin);
    }
    return status;
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
extern "C" int DigipasDWLConfig(const char* conn_port, const char* driver_port, const char* mode, int country, int city) {
    DigipasDWL* pDigipasDWL = new DigipasDWL(conn_port, driver_port, mode, country, city);
    pDigipasDWL = NULL;
    return (asynSuccess);
}

static const iocshArg DigipasDWLArg0 = {"Connection asyn port", iocshArgString};
static const iocshArg DigipasDWLArg1 = {"Driver asyn port", iocshArgString};
static const iocshArg DigipasDWLArg2 = {"Mode", iocshArgString};
static const iocshArg DigipasDWLArg3 = {"Country code", iocshArgInt};
static const iocshArg DigipasDWLArg4 = {"City code", iocshArgInt};
static const iocshArg* const DigipasDWLArgs[5] = {&DigipasDWLArg0, &DigipasDWLArg1, &DigipasDWLArg2,
                                                  &DigipasDWLArg3, &DigipasDWLArg4};
static const iocshFuncDef DigipasDWLFuncDef = {"DigipasDWLConfig", 4, DigipasDWLArgs};

static void DigipasDWLCallFunc(const iocshArgBuf* args) {
    DigipasDWLConfig(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival);
}

void DigipasDWLRegister(void) { iocshRegister(&DigipasDWLFuncDef, DigipasDWLCallFunc); }

extern "C" {
epicsExportRegistrar(DigipasDWLRegister);
}
