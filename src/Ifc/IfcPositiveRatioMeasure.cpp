/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcSizeSelect.h"
#include "ifcpp/Ifc/IfcPositiveRatioMeasure.h"

// TYPE IfcPositiveRatioMeasure = IfcRatioMeasure;
void IFC4X3::IfcPositiveRatioMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCPOSITIVERATIOMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcPositiveRatioMeasure> IFC4X3::IfcPositiveRatioMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcPositiveRatioMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcPositiveRatioMeasure>(); }
	shared_ptr<IfcPositiveRatioMeasure> type_object( new IfcPositiveRatioMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}