#include "atri_detector/onnx_inference.hpp"

// C++
#include <iostream>
#include <memory>
#include <string>

namespace atri_detector
{

bool OnnxInference::loadOnnx(const std::string & onnx_path)
{
  if (onnx_path.empty()) {
    std::cerr << "[OnnxInference] ONNX model path is empty!" << std::endl;
    return false;
  }

  try {
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    try {
      OrtCUDAProviderOptions cuda_options;
      cuda_options.device_id = 0;
      cuda_options.arena_extend_strategy = 0;
      cuda_options.gpu_mem_limit = static_cast<size_t>(2ULL * 1024 * 1024 * 1024);
      cuda_options.cudnn_conv_algo_search = OrtCudnnConvAlgoSearchExhaustive;
      cuda_options.do_copy_in_default_stream = 1;
      session_options.AppendExecutionProvider_CUDA(cuda_options);
      std::cout << "[OnnxInference] CUDA Execution Provider enabled." << std::endl;
    } catch (const Ort::Exception & e) {
      std::cerr << "[OnnxInference] CUDA EP not available, using CPU: " << e.what() << std::endl;
    }

    session_ = std::make_unique<Ort::Session>(env_, onnx_path.c_str(), session_options);
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::AllocatorWithDefaultOptions allocator;
    auto input_name_ptr = session_->GetInputNameAllocated(0, allocator);
    input_name_ = std::string(input_name_ptr.get());

    auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    input_dims_.clear();
    input_size_ = 1;
    for (auto dim : input_shape) {
      int d = (dim < 0) ? 1 : static_cast<int>(dim);
      input_dims_.push_back(d);
      input_size_ *= static_cast<size_t>(d);
    }

    auto output_name_ptr = session_->GetOutputNameAllocated(0, allocator);
    output_name_ = std::string(output_name_ptr.get());

    auto output_shape = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    output_dims_.clear();
    output_size_ = 1;
    for (auto dim : output_shape) {
      int d = (dim < 0) ? 1 : static_cast<int>(dim);
      output_dims_.push_back(d);
      output_size_ *= static_cast<size_t>(d);
    }

    std::cout << "[OnnxInference] Model loaded: " << onnx_path << std::endl;
    std::cout << "[OnnxInference] Input: " << input_name_ << std::endl;
    std::cout << "[OnnxInference] Output: " << output_name_ << std::endl;

    return true;
  } catch (const Ort::Exception & e) {
    std::cerr << "[OnnxInference] Failed to load model: " << e.what() << std::endl;
    return false;
  }
}

bool OnnxInference::infer(const std::vector<float> & input, std::vector<float> & output)
{
  if (input.size() != input_size_) {
    std::cerr << "[OnnxInference] Input size mismatch: expected " << input_size_ << ", got "
              << input.size() << std::endl;
    return false;
  }

  try {
    std::vector<int64_t> input_shape(input_dims_.begin(), input_dims_.end());
    auto input_tensor = Ort::Value::CreateTensor<float>(
      memory_info_, const_cast<float *>(input.data()), input_size_, input_shape.data(),
      input_shape.size());

    const char * input_names[] = {input_name_.c_str()};
    const char * output_names[] = {output_name_.c_str()};

    auto output_tensors =
      session_->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    float * output_data = output_tensors[0].GetTensorMutableData<float>();
    size_t actual_output_size = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();

    output.assign(output_data, output_data + actual_output_size);

    return true;
  } catch (const Ort::Exception & e) {
    std::cerr << "[OnnxInference] Inference failed: " << e.what() << std::endl;
    return false;
  }
}

}  // namespace atri_detector
