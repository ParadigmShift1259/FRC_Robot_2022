#include "common/DebugFlag.h"

int DebugFlag::m_flagCount = 0;

const int c_numColumns = 7;

DebugFlag::DebugFlag(const char* name, bool defaultVal) : m_defaultVal(defaultVal)
{
    frc::ShuffleboardTab& tab = frc::Shuffleboard::GetTab("DebugFlags");

    int row = m_flagCount / c_numColumns;
    int col = m_flagCount % c_numColumns;
    m_netTableEntry = tab.Add(name, m_defaultVal)
                         .WithWidget(frc::BuiltInWidgets::kToggleSwitch)
                         .WithSize(1, 1)
                         .WithPosition(row, col)
                         .GetEntry();
    m_flagCount++;
}
