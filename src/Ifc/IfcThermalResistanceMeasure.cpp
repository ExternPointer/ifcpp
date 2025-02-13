/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcThermalResistanceMeasure.h"

// TYPE IfcThermalResistanceMeasure = REAL;
IFC4X3::IfcThermalResistanceMeasure::IfcThermalResistanceMeasure( double value ) { m_value = value; }
void IFC4X3::IfcThermalResistanceMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCTHERMALRESISTANCEMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcThermalResistanceMeasure> IFC4X3::IfcThermalResistanceMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcThermalResistanceMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcThermalResistanceMeasure>(); }
	shared_ptr<IfcThermalResistanceMeasure> type_object( new IfcThermalResistanceMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}
