#!/bin/bash

sudo apt -y update \
    && apt install -y ccache \
       clang-tidy-12 \
       cppcheck \
       graphviz \
       doxygen \
       libeigen3-dev
