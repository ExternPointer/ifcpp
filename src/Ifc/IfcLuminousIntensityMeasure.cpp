/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcLuminousIntensityMeasure.h"

// TYPE IfcLuminousIntensityMeasure = REAL;
IFC4X3::IfcLuminousIntensityMeasure::IfcLuminousIntensityMeasure( double value ) { m_value = value; }
void IFC4X3::IfcLuminousIntensityMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCLUMINOUSINTENSITYMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcLuminousIntensityMeasure> IFC4X3::IfcLuminousIntensityMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcLuminousIntensityMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcLuminousIntensityMeasure>(); }
	shared_ptr<IfcLuminousIntensityMeasure> type_object( new IfcLuminousIntensityMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}
