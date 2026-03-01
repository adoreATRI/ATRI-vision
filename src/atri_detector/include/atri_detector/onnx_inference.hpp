#ifndef ATRI_DETECTOR__ONNX_INFERENCE_HPP_
#define ATRI_DETECTOR__ONNX_INFERENCE_HPP_

#include <onnxruntime_cxx_api.h>

#include <vector>

namespace atri_detector
{

class OnnxInference
{
public:
  OnnxInference() = default;
  ~OnnxInference() = default;

  bool loadOnnx(const std::string & onnx_path = "");

  bool infer(const std::vector<float> & input, std::vector<float> & output);

  std::vector<int> getInputDims() const { return input_dims_; }
  std::vector<int> getOutputDims() const { return output_dims_; }

private:
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "atri_detector"};
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_info_{nullptr};

  std::string input_name_;
  std::string output_name_;
  std::vector<int> input_dims_;
  std::vector<int> output_dims_;
  size_t input_size_ = 0;
  size_t output_size_ = 0;
};

}  // namespace atri_detector

#endif  // ATRI_DETECTOR__ONNX_INFERENCE_HPP_
