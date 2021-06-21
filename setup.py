from setuptools import setup, find_packages

setup(
    name='txmoptics',
    version=open('VERSION').read().strip(),
    author='Viktor Nikitin, Francesco De Carlo',
    url='https://github.com/nikitinvv/txmoptics',
    packages=find_packages(),
    include_package_data = True,
    description='Module to control TXM optics at sector 32id',
    zip_safe=False,
)