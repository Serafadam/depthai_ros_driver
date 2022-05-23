#!/bin/bash
ament_uncrustify src/ && ament_cpplint src/ && ament_cppcheck src/ && ament_lint_cmake src/ && ament_pep257 src/ && ament_xmllint src/
