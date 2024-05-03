# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# configurations for PDF output by Read the Docs
project = "HIPCC Documentation"
author = "Advanced Micro Devices, Inc."
copyright = "Copyright (c) 2024 Advanced Micro Devices, Inc. All rights reserved."
version = "5.7.0"
version = "5.7.0"

exclude_patterns = ['temp']

external_toc_path = "./sphinx/_toc.yml"

extensions = ["rocm_docs"]

external_projects_current_project = "hipcc"

html_theme = "rocm_docs_theme"
html_theme_options = {"flavor": "rocm-docs-home"}

html_title = "HIPCC Documentation"

html_theme_options = {
    "link_main_doc": False
}
