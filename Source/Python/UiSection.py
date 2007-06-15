import Section
from Ffs import Ffs
class UiSection (Section.Section):
    def __init__(self):
        self.Alignment = None
        self.StringData = None
        self.FileName = None


    def GenSection(self, OutputPath, ModuleName):
        #
        # Prepare the parameter of GenSection
        #
        OutputFile = OutputPath + ModuleName + Ffs.SectionSuffix('UI')
        GenSectionCmd = 'GenSection -o ' + OutputFile + ' -s EFI_SECTION_USER_INTERFACE ' \
                         + '-n ' + '\"' + self.StringData + '\"'
        #
        # Call GenSection
        #
        popen(GenSectionCmd, mod = 'r')

        return OutputFile
