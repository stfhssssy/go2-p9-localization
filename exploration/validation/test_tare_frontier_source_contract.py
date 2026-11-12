#!/usr/bin/env python3
"""Check patched TARE source and compile exact parameter-reader preambles.

Usage: python3 test_tare_frontier_source_contract.py /tare_ws
Creates no ROS graph. Actual ROS and closed-loop tests remain separate.
"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


def check(workspace):
    source = workspace / 'src/tare_planner/src'
    main = source / 'sensor_coverage_planner/sensor_coverage_planner_ground.cpp'
    text = main.read_text()
    declaration = 'this->declare_parameter<bool>("kUseFrontier", true);'
    assert text.count(declaration) == 1, 'Expected one explicit true-default declaration'
    read = text[text.index('void SensorCoveragePlanner3D::ReadParameters()'):]
    assert read.index(declaration) < read.index('void SensorCoveragePlanner3D::InitializeData()')
    initialize = text[text.index('bool SensorCoveragePlanner3D::initialize()'):]
    assert initialize.index('ReadParameters();') < initialize.index('InitializeData();')
    declared, reads = set(), set()
    for path in source.rglob('*.cpp'):
        content = path.read_text()
        declared.update(re.findall(r'declare_parameter(?:<[^>]+>)?\(\s*"([^\"]+)"', content))
        reads.update(re.findall(r'get_parameter\(\s*"([^\"]+)"', content))
    assert not reads-declared, 'Undeclared literal parameter reads: '+repr(reads-declared)
    preambles = []
    for relative in ['viewpoint_manager/viewpoint_manager.cpp', 'planning_env/planning_env.cpp']:
        content = (source/relative).read_text()
        start = content.index('  // An absent declaration must stop startup, never leave an indeterminate bool.')
        end = content.index('\n  }', start)+len('\n  }')
        preamble = content[start:end]
        assert 'kUseFrontier = false;' in preamble
        assert 'if (!nh->get_parameter("kUseFrontier", kUseFrontier))' in preamble
        assert 'throw std::runtime_error' in preamble
        assert '#include <stdexcept>' in content
        preambles.append(preamble)
    compiler = shutil.which('c++') or shutil.which('g++')
    assert compiler, 'C++ compiler required for exact reader preambles'
    cpp = '''#include <memory>
#include <stdexcept>
#include <string>
struct ParameterNode {
  bool declared, configured_value;
  bool get_parameter(const std::string& key, bool& result) {
    if (key != "kUseFrontier" || !declared) return false;
    result = configured_value;
    return true;
  }
};
'''
    for index, preamble in enumerate(preambles):
        cpp += 'struct Reader'+str(index)+' { bool kUseFrontier = true; void read(std::shared_ptr<ParameterNode> nh) {\n'+preamble+'\n} };\n'
    cpp += '''template<class Reader> void exercise() {
  for (bool value : {false, true}) {
    Reader reader;
    reader.read(std::make_shared<ParameterNode>(ParameterNode{true, value}));
    if (reader.kUseFrontier != value) throw std::runtime_error("override not honored");
  }
  Reader reader;
  bool threw = false;
  try { reader.read(std::make_shared<ParameterNode>(ParameterNode{false, true})); }
  catch (const std::runtime_error&) { threw = true; }
  if (!threw || reader.kUseFrontier) throw std::runtime_error("missing parameter did not fail closed");
}
int main() { exercise<Reader0>(); exercise<Reader1>(); }
'''
    with tempfile.TemporaryDirectory(prefix='p9-frontier-contract-') as directory:
        root = Path(directory)
        (root/'readers.cpp').write_text(cpp)
        subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        str(root/'readers.cpp'), '-o', str(root/'readers')], check=True, timeout=30)
        subprocess.run([str(root/'readers')], check=True, timeout=5)
    print('PASS: declared before readers; all literal reads declared; exact readers honor true/false and reject missing')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workspace', type=Path)
    check(parser.parse_args().workspace.resolve())
