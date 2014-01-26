from setuptools import setup, find_packages

setup(name='motmot.trackem',
      description='multiple realtime point tracker (part of motmot)',
      version='0.5',
      entry_points = {
    'motmot.fview.plugins':'trackem = motmot.trackem.trackem:TrackemClass',
    },
      namespace_packages = ['motmot'],
      packages = find_packages(),
      author='Andrew Straw',
      author_email='strawman@astraw.com',
      url='http://code.astraw.com/projects/motmot',
      zip_safe=False,
      package_data = {'motmot.trackem':['*.xrc',
                                        '*.m']},
      )
