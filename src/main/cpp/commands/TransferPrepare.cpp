#include "commands/TransferPrepare.h"
#include "Constants.h"

using namespace TransferConstants;

TransferPrepare::TransferPrepare(TransferSubsystem* subsystem)
 : m_transfer(subsystem)
{
    AddRequirements({subsystem});
}
void TransferPrepare::Initialize() {
    m_transfer->StartDetection();
}

void TransferPrepare::Execute() {
    m_transfer->SetTransfer(kTransferSpeedFiring);
    m_transfer->SetFeeder(0);
}

// Set to true immediately because sensor was removed
bool TransferPrepare::IsFinished() {
    return m_transfer->AtFeederPosition();
}

void TransferPrepare::End(bool interrupted) {
    m_transfer->SetFeeder(0);
    m_transfer->SetTransfer(0);

}