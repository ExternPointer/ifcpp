/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcHatchLineDistanceSelect.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcSizeSelect.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"

// TYPE IfcPositiveLengthMeasure = IfcLengthMeasure;
void IFC4X3::IfcPositiveLengthMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCPOSITIVELENGTHMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcPositiveLengthMeasure> IFC4X3::IfcPositiveLengthMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcPositiveLengthMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcPositiveLengthMeasure>(); }
	shared_ptr<IfcPositiveLengthMeasure> type_object( new IfcPositiveLengthMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}