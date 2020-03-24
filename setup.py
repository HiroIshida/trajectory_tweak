from setuptools import setup, find_packages
setup(
    name='trajectory',
    version='0.0.1',
    description='Sample package for Python-Guide.org',
    license=license,
    #install_requires=['numpy', 'scipy', 'scikit-image', 'sklearn', 'pyclustering', 'torch', 'gpytorch'],
    packages=find_packages(exclude=('tests', 'docs'))
)
