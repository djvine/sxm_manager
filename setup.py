try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

config = {
    'description': 'Scanning X-ray Microscope Manager',
    'author': 'David J. Vine',
    'url': 'URL to get it at.',
    'download_url': 'Where to download it.',
    'author_email': 'djvine@gmail.com',
    'version': '0.1',
    'install_requires': [
        'nose',
        'numpy',
        'scipy',
        'pyepics'],
    'packages': ['sxm_manager'],
    'scripts': [],
    'name': 'sxm_manager'
}

setup(**config)
