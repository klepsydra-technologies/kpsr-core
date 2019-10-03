# Klepsydra Core Modules
# Copyright (C) 2019-2020  Klepsydra Technologies GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from setuptools import find_packages, setup

setup(
    name='kpsr_codegen',
    version='1.0',
    packages=find_packages(),
    package_data={
        'kpsr_codegen': ['conf/*.yaml', 'templates/*.*'],
    },
    include_package_data=True,
    python_requires='>=3.6',
    install_requires=[
        'setuptools',
        'nose',
        'coverage',
        'lcov_cobertura',
        'Jinja2',
        'PyYaml'
    ],
    entry_points='''
        [console_scripts]
        kpsr_codegen = kpsr_codegen.__main__:main
    ''',
)
