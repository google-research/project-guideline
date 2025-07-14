// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "absl/status/status.h"
#include "mediapipe/framework/calculator_framework.h"
#include "mediapipe/framework/port/ret_check.h"

namespace guideline {

namespace {

using ::mediapipe::CalculatorBase;
using ::mediapipe::CalculatorContext;
using ::mediapipe::CalculatorContract;

}  // namespace

// Converts bool to an int.
class BoolToIntCalculator : public CalculatorBase {
 public:
  static absl::Status GetContract(CalculatorContract* cc) {
    RET_CHECK_EQ(cc->Inputs().NumEntries(), 1);
    RET_CHECK_EQ(cc->Outputs().NumEntries(), 1);
    cc->Inputs().Index(0).Set<bool>();
    cc->Outputs().Index(0).Set<int>();
    return absl::OkStatus();
  }

  absl::Status Process(CalculatorContext* cc) override {
    const int output = cc->Inputs().Index(0).Get<bool>() ? 1 : 0;
    cc->Outputs().Index(0).AddPacket(
        mediapipe::MakePacket<int>(output).At(cc->InputTimestamp()));
    return absl::OkStatus();
  }
};
REGISTER_CALCULATOR(BoolToIntCalculator);

}  // namespace guideline
