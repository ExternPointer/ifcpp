/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAreaMeasure.h"
#include "ifcpp/Ifc/IfcBendingParameterSelect.h"
#include "ifcpp/Ifc/IfcGloballyUniqueId.h"
#include "ifcpp/Ifc/IfcIdentifier.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOwnerHistory.h"
#include "ifcpp/Ifc/IfcPositiveLengthMeasure.h"
#include "ifcpp/Ifc/IfcPropertySetDefinition.h"
#include "ifcpp/Ifc/IfcReinforcingBarSurfaceEnum.h"
#include "ifcpp/Ifc/IfcReinforcingBarType.h"
#include "ifcpp/Ifc/IfcReinforcingBarTypeEnum.h"
#include "ifcpp/Ifc/IfcRelAggregates.h"
#include "ifcpp/Ifc/IfcRelAssigns.h"
#include "ifcpp/Ifc/IfcRelAssignsToProduct.h"
#include "ifcpp/Ifc/IfcRelAssociates.h"
#include "ifcpp/Ifc/IfcRelDeclares.h"
#include "ifcpp/Ifc/IfcRelDefinesByType.h"
#include "ifcpp/Ifc/IfcRelNests.h"
#include "ifcpp/Ifc/IfcRepresentationMap.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcReinforcingBarType 
IFC4X3::IfcReinforcingBarType::IfcReinforcingBarType( int tag ) { m_tag = tag; }
void IFC4X3::IfcReinforcingBarType::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCREINFORCINGBARTYPE" << "(";
	if( m_GlobalId ) { m_GlobalId->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_OwnerHistory ) { stream << "#" << m_OwnerHistory->m_tag; } else { stream << "$"; }
	stream << ",";
	if( m_Name ) { m_Name->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ApplicableOccurrence ) { m_ApplicableOccurrence->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	writeEntityList( stream, m_HasPropertySets );
	stream << ",";
	writeEntityList( stream, m_RepresentationMaps );
	stream << ",";
	if( m_Tag ) { m_Tag->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ElementType ) { m_ElementType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_PredefinedType ) { m_PredefinedType->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_NominalDiameter ) { m_NominalDiameter->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_CrossSectionArea ) { m_CrossSectionArea->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_BarLength ) { m_BarLength->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_BarSurface ) { m_BarSurface->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_BendingShapeCode ) { m_BendingShapeCode->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_BendingParameters.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_BendingParameters.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcBendingParameterSelect>& type_object = m_BendingParameters[ii];
			if( type_object )
			{
				type_object->getStepParameter( stream, true );
			}
			else
			{
				stream << "$";
			}
		}
		stream << ")";
	}
	else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcReinforcingBarType::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcReinforcingBarType::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 16 ){ std::stringstream err; err << "Wrong parameter count for entity IfcReinforcingBarType, expecting 16, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_GlobalId = IfcGloballyUniqueId::createObjectFromSTEP( args[0], map, errorStream );
	readEntityReference( args[1], m_OwnerHistory, map, errorStream );
	m_Name = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[3], map, errorStream );
	m_ApplicableOccurrence = IfcIdentifier::createObjectFromSTEP( args[4], map, errorStream );
	readEntityReferenceList( args[5], m_HasPropertySets, map, errorStream );
	readEntityReferenceList( args[6], m_RepresentationMaps, map, errorStream );
	m_Tag = IfcLabel::createObjectFromSTEP( args[7], map, errorStream );
	m_ElementType = IfcLabel::createObjectFromSTEP( args[8], map, errorStream );
	m_PredefinedType = IfcReinforcingBarTypeEnum::createObjectFromSTEP( args[9], map, errorStream );
	m_NominalDiameter = IfcPositiveLengthMeasure::createObjectFromSTEP( args[10], map, errorStream );
	m_CrossSectionArea = IfcAreaMeasure::createObjectFromSTEP( args[11], map, errorStream );
	m_BarLength = IfcPositiveLengthMeasure::createObjectFromSTEP( args[12], map, errorStream );
	m_BarSurface = IfcReinforcingBarSurfaceEnum::createObjectFromSTEP( args[13], map, errorStream );
	m_BendingShapeCode = IfcLabel::createObjectFromSTEP( args[14], map, errorStream );
	readSelectList( args[15], m_BendingParameters, map, errorStream );
}
void IFC4X3::IfcReinforcingBarType::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcReinforcingElementType::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "PredefinedType", m_PredefinedType ) );
	vec_attributes.emplace_back( std::make_pair( "NominalDiameter", m_NominalDiameter ) );
	vec_attributes.emplace_back( std::make_pair( "CrossSectionArea", m_CrossSectionArea ) );
	vec_attributes.emplace_back( std::make_pair( "BarLength", m_BarLength ) );
	vec_attributes.emplace_back( std::make_pair( "BarSurface", m_BarSurface ) );
	vec_attributes.emplace_back( std::make_pair( "BendingShapeCode", m_BendingShapeCode ) );
	shared_ptr<AttributeObjectVector> BendingParameters_vec_object( new AttributeObjectVector() );
	std::copy( m_BendingParameters.begin(), m_BendingParameters.end(), std::back_inserter( BendingParameters_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "BendingParameters", BendingParameters_vec_object ) );
}
void IFC4X3::IfcReinforcingBarType::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcReinforcingElementType::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcReinforcingBarType::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcReinforcingElementType::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcReinforcingBarType::unlinkFromInverseCounterparts()
{
	IfcReinforcingElementType::unlinkFromInverseCounterparts();
}
