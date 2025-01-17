#include "graphics/glslang_compiler.h"

#include <cstring>
#include <format>
#include <iostream>
#include <memory>
#include <print>
#include <stdexcept>
#include <string_view>
#include <utility>

#include <glslang/Include/glslang_c_interface.h>
#include <glslang/Public/resource_limits_c.h>

template <>
struct std::formatter<glslang_stage_t> : std::formatter<std::string_view> {
  [[nodiscard]] auto format(const glslang_stage_t stage, std::format_context& format_context) const {
    return std::formatter<std::string_view>::format(to_string(stage), format_context);
  }

private:
  static constexpr std::string_view to_string(const glslang_stage_t stage) noexcept {
    switch (stage) {
      // clang-format off
#define CASE(kGlslangStage) case kGlslangStage: return #kGlslangStage;  // NOLINT(cppcoreguidelines-macro-usage)
      CASE(GLSLANG_STAGE_VERTEX)
      CASE(GLSLANG_STAGE_TESSCONTROL)
      CASE(GLSLANG_STAGE_TESSEVALUATION)
      CASE(GLSLANG_STAGE_GEOMETRY)
      CASE(GLSLANG_STAGE_FRAGMENT)
      CASE(GLSLANG_STAGE_COMPUTE)
      CASE(GLSLANG_STAGE_RAYGEN)
      CASE(GLSLANG_STAGE_INTERSECT)
      CASE(GLSLANG_STAGE_ANYHIT)
      CASE(GLSLANG_STAGE_CLOSESTHIT)
      CASE(GLSLANG_STAGE_MISS)
      CASE(GLSLANG_STAGE_CALLABLE)
      CASE(GLSLANG_STAGE_TASK)
      CASE(GLSLANG_STAGE_MESH)
      CASE(GLSLANG_STAGE_COUNT)
#undef CASE
        // clang-format on
      default:
        std::unreachable();
    }
  }
};

namespace {

using UniqueGlslangShader = std::unique_ptr<glslang_shader_t, decltype(&glslang_shader_delete)>;
using UniqueGlslangProgram = std::unique_ptr<glslang_program_t, decltype(&glslang_program_delete)>;

constexpr auto kGlslangMessages =
// NOLINTBEGIN(hicpp-signed-bitwise)
#ifndef NDEBUG
    GLSLANG_MSG_DEBUG_INFO_BIT |
#endif
    GLSLANG_MSG_SPV_RULES_BIT | GLSLANG_MSG_VULKAN_RULES_BIT;
// NOLINTEND(hicpp-signed-bitwise)

std::ostream& operator<<(std::ostream& ostream, glslang_shader_t* const shader) {
  if (const auto* const shader_info_log = glslang_shader_get_info_log(shader);
      shader_info_log != nullptr && std::strlen(shader_info_log) > 0) {
    std::println(ostream, "{}", shader_info_log);
  }
#ifndef NDEBUG
  if (const auto* const shader_info_debug_log = glslang_shader_get_info_debug_log(shader);
      shader_info_debug_log != nullptr && std::strlen(shader_info_debug_log) > 0) {
    std::println(ostream, "{}", shader_info_debug_log);
  }
#endif
  return ostream;
}

std::ostream& operator<<(std::ostream& ostream, glslang_program_t* const program) {
  if (const auto* const program_info_log = glslang_program_get_info_log(program);
      program_info_log != nullptr && std::strlen(program_info_log) > 0) {
    std::println(ostream, "{}", program_info_log);
  }
#ifndef NDEBUG
  if (const auto* const program_info_debug_log = glslang_program_get_info_debug_log(program);
      program_info_debug_log != nullptr && std::strlen(program_info_debug_log) > 0) {
    std::println(ostream, "{}", program_info_debug_log);
  }
#endif
  return ostream;
}

UniqueGlslangShader CreateShader(const glslang_stage_t stage, const char* const glsl_source) {
  const glslang_input_t input{.language = GLSLANG_SOURCE_GLSL,
                              .stage = stage,
                              .client = GLSLANG_CLIENT_VULKAN,
                              .client_version = GLSLANG_TARGET_VULKAN_1_3,
                              .target_language = GLSLANG_TARGET_SPV,
                              .target_language_version = GLSLANG_TARGET_SPV_1_6,
                              .code = glsl_source,
                              .default_version = 460,
                              .default_profile = GLSLANG_NO_PROFILE,
                              .force_default_version_and_profile = 0,
                              .forward_compatible = 0,
                              .messages = static_cast<glslang_messages_t>(kGlslangMessages),
                              .resource = glslang_default_resource()};

  auto shader = UniqueGlslangShader{glslang_shader_create(&input), glslang_shader_delete};
  if (shader == nullptr) {
    throw std::runtime_error{std::format("Shader creation failed at {} with GLSL source:\n{}", stage, glsl_source)};
  }

  const auto shader_preprocess_status = glslang_shader_preprocess(shader.get(), &input);
  std::clog << shader.get();

  if (shader_preprocess_status == 0) {
    throw std::runtime_error{
        std::format("Shader preprocessing failed at {} with GLSL source:\n{}", stage, glsl_source)};
  }

  const auto shader_parse_status = glslang_shader_parse(shader.get(), &input);
  std::clog << shader.get();

  if (shader_parse_status == 0) {
    throw std::runtime_error{std::format("Shader parsing failed at {} with GLSL source:\n{}",
                                         stage,
                                         glslang_shader_get_preprocessed_code(shader.get()))};
  }

  return shader;
}

UniqueGlslangProgram CreateProgram(const glslang_stage_t stage, glslang_shader_t* const shader) {
  auto program = UniqueGlslangProgram{glslang_program_create(), glslang_program_delete};
  if (program == nullptr) throw std::runtime_error{std::format("Shader program creation failed at {}", stage)};

  glslang_program_add_shader(program.get(), shader);

  const auto program_link_status = glslang_program_link(program.get(), kGlslangMessages);
  std::clog << program.get();

  if (program_link_status == 0) {
    throw std::runtime_error{std::format("Shader program linking failed at {} with GLSL source:\n{}",
                                         stage,
                                         glslang_shader_get_preprocessed_code(shader))};
  }

  return program;
}

std::vector<std::uint32_t> GenerateSpirv(glslang_program_t* const program, const glslang_stage_t stage) {
  glslang_program_SPIRV_generate(program, stage);

  const auto spirv_size = glslang_program_SPIRV_get_size(program);
  if (spirv_size == 0) throw std::runtime_error{std::format("SPIR-V generation failed at {}", stage)};

  std::vector<std::uint32_t> spirv(spirv_size);
  glslang_program_SPIRV_get(program, spirv.data());

#ifndef NDEBUG
  if (const auto* const spirv_messages = glslang_program_SPIRV_get_messages(program);
      spirv_messages != nullptr && std::cmp_greater(std::strlen(spirv_messages), 0)) {
    std::println(std::clog, "{}", spirv_messages);
  }
#endif

  return spirv;
}

}  // namespace

namespace gfx {

GlslangCompiler::GlslangCompiler() {
  if (glslang_initialize_process() == 0) {
    throw std::runtime_error{"glslang initialization failed"};
  }
}

GlslangCompiler::~GlslangCompiler() noexcept { glslang_finalize_process(); }

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
std::vector<std::uint32_t> GlslangCompiler::Compile(const glslang_stage_t stage, const char* const glsl_source) const {
  const auto shader = CreateShader(stage, glsl_source);
  const auto program = CreateProgram(stage, shader.get());
  return GenerateSpirv(program.get(), stage);
}

}  // namespace gfx
