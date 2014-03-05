# Requires wxPython.  This sample demonstrates:
#
# - single file exe using wxPython as GUI.

from distutils.core import setup
import py2exe
import sys
import matplotlib

# If run without args, build executables, in quiet mode.
if len(sys.argv) == 1:
    sys.argv.append("py2exe")
    sys.argv.append("-q")

class Target:
    def __init__(self, **kw):
        self.__dict__.update(kw)
        # for the versioninfo resources
        self.version = "0.6.1"
        self.company_name = "Tomorrow"
        self.copyright = "GPL Copyrights"
        self.name = "Simulation system"

################################################################
# A program using wxPython

# The manifest will be inserted as resource into test_wx.exe.  This
# gives the controls the Windows XP appearance (if run on XP ;-)
#
# Another option would be to store it in a file named
# test_wx.exe.manifest, and copy it with the data_files option into
# the dist-dir.
#
manifest_template = '''
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<assembly xmlns="urn:schemas-microsoft-com:asm.v1" manifestVersion="1.0">
<assemblyIdentity
    version="5.0.0.0"
    processorArchitecture="x86"
    name="%(prog)s"
    type="win32"
/>
<description>%(prog)s Program</description>
<dependency>
    <dependentAssembly>
        <assemblyIdentity
            type="win32"
            name="Microsoft.VC90.CRT"
            version="9.0.30729.6161"
            processorArchitecture="x86"
            publicKeyToken="1fc8b3b9a1e18e3b"
            language="*"
        />
    </dependentAssembly>
</dependency>
</assembly>
'''

RT_MANIFEST = 24

PlotSerialData = Target(
    # used for the versioninfo resource
    description = "Simulation system",

    # what to build
    script = "PlotSerialData.py",
    other_resources = [(RT_MANIFEST, 1, manifest_template % dict(prog="NewPlotSerial"))],
    icon_resources = [(1, "network.ico")],
    dest_base = "PlotSerialData")

################################################################

setup(
    options = {"py2exe": {"compressed": 1,
                          "optimize": 2,
                          "ascii": 1,
                          "bundle_files": 1,
                          "includes": ['matplotlib.figure', 'matplotlib.backends.backend_wxagg', 'numpy', 'pylab'],
                          'excludes': ['_gtkagg', '_tkagg', '_agg2', '_cairo', '_cocoaagg','_fltkagg', '_gtk', '_gtkcairo'],
                          "dll_excludes": ['libgdk-win32-2.0-0.dll','libgobject-2.0-0.dll']}
               },
    zipfile = None,
    data_files = matplotlib.get_py2exe_datafiles(),
    windows = [PlotSerialData],
    )