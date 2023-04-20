/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcModulusOfTranslationalSubgradeReactionSelect.h"
#include "ifcpp/Ifc/IfcModulusOfLinearSubgradeReactionMeasure.h"

// TYPE IfcModulusOfLinearSubgradeReactionMeasure = REAL;
IFC4X3::IfcModulusOfLinearSubgradeReactionMeasure::IfcModulusOfLinearSubgradeReactionMeasure( double value ) { m_value = value; }
void IFC4X3::IfcModulusOfLinearSubgradeReactionMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCMODULUSOFLINEARSUBGRADEREACTIONMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcModulusOfLinearSubgradeReactionMeasure> IFC4X3::IfcModulusOfLinearSubgradeReactionMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcModulusOfLinearSubgradeReactionMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcModulusOfLinearSubgradeReactionMeasure>(); }
	shared_ptr<IfcModulusOfLinearSubgradeReactionMeasure> type_object( new IfcModulusOfLinearSubgradeReactionMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}