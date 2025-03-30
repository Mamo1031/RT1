# Configuration file for the Sphinx documentation builder.
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# -- Path setup --------------------------------------------------------------
# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute.
sys.path.insert(0, os.path.abspath('scripts'))

# -- Project information -----------------------------------------------------
project = 'assignment1_rt'
author = 'Mamoru Ota'
copyright = '2025, Mamoru Ota'

# -- General configuration ---------------------------------------------------
extensions = ['sphinx.ext.autodoc']
templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# autodoc configuration - Mock non-existent modules
autodoc_mock_imports = ['turtlesim', 'rospy', 'geometry_msgs', 'other_missing_modules']

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_output_path = 'docs'
