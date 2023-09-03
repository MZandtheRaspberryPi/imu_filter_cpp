#include "filter_base.h"

m_t do_cos(const m_t& angle) { return cos(angle); }
m_t do_sin(const m_t& angle) { return sin(angle); }
m_t do_tan(const m_t& angle) { return tan(angle); }
m_t do_arctan(const m_t& y, const m_t& x) { return atan2(y, x); }
