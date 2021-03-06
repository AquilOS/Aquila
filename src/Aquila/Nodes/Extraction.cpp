#include "Aquila/Nodes/Extraction.h"
using namespace aq;
using namespace aq::Nodes;

TS<SyncedMemory> CpuExtraction::process(TS<SyncedMemory> input, cv::cuda::Stream& stream)
{
    process(input.GetMat(stream), input.timestamp, input.frame_number, stream);
    return input;
}
TS<SyncedMemory> GpuExtraction::process(TS<SyncedMemory> input, cv::cuda::Stream& stream)
{
    process(input.GetGpuMat(stream), input.timestamp, input.frame_number, stream);
    return input;
}
