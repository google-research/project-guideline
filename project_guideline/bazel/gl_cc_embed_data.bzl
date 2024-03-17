# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Embeds binary data in cc_*() rules."""

_FILEWRAPPER = "//project_guideline/bazel/tools/filewrapper"

def _gl_cc_embed_data_impl(ctx):
    cc_file_artifact = None
    h_file_artifact = None
    for output in ctx.outputs.outs:
        if output.path.endswith(".h"):
            h_file_artifact = output
        elif output.path.endswith(".cc") or output.path.endswith(".cpp"):
            cc_file_artifact = output

    args = ctx.actions.args()
    args.add(ctx.label.package)
    args.add(ctx.attr.ident)
    args.add(ctx.attr.namespace if ctx.attr.namespace else "")
    args.add(h_file_artifact)
    args.add(cc_file_artifact)
    args.add_all(ctx.files.srcs)

    ctx.actions.run(
        executable = ctx.executable._filewrapper,
        inputs = ctx.files.srcs,
        outputs = [h_file_artifact, cc_file_artifact],
        arguments = [args],
        mnemonic = "CcEmbedData",
        progress_message = (
            "Creating gl_cc_embed_data file for {}".format(ctx.label)
        ),
    )

_gl_cc_embed_data = rule(
    implementation = _gl_cc_embed_data_impl,
    attrs = {
        "srcs": attr.label_list(
            allow_files = True,
        ),
        "namespace": attr.string(),
        "ident": attr.string(),
        "_filewrapper": attr.label(
            executable = True,
            cfg = "exec",
            allow_files = True,
            default = Label(_FILEWRAPPER),
        ),
        "outs": attr.output_list(),
    },
)

def gl_cc_embed_data(name, srcs = [], namespace = "", **kwargs):
    """Embeds arbitrary binary data in cc_*() rules.

    Args:
      name: Name for this rule.
      srcs: A list of files to be embedded.
      namespace: C++ namespace to wrap the generated types in.
      **kwargs: extra arguments like testonly, visibility, etc.
    """
    embed_rule = "_%s_gl" % name
    _gl_cc_embed_data(
        name = embed_rule,
        srcs = srcs,
        namespace = namespace,
        ident = name,
        outs = [
            "%s.h" % name,
            "%s.cc" % name,
        ],
    )
    native.cc_library(
        name = name,
        hdrs = [":%s.h" % name],
        srcs = [":%s.cc" % name],
        deps = [
            "@com_google_absl//absl/base:core_headers",
            "@com_google_absl//absl/strings",
        ],
        **kwargs
    )
