/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <sstream>
#include <limits>

#include "ifcpp/Model/AttributeObject.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Model/BuildingGuid.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Ifc/IfcAddressTypeEnum.h"
#include "ifcpp/Ifc/IfcLabel.h"
#include "ifcpp/Ifc/IfcOrganization.h"
#include "ifcpp/Ifc/IfcPerson.h"
#include "ifcpp/Ifc/IfcPostalAddress.h"
#include "ifcpp/Ifc/IfcText.h"

// ENTITY IfcPostalAddress 
IFC4X3::IfcPostalAddress::IfcPostalAddress( int tag ) { m_tag = tag; }
void IFC4X3::IfcPostalAddress::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCPOSTALADDRESS" << "(";
	if( m_Purpose ) { m_Purpose->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedPurpose ) { m_UserDefinedPurpose->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_InternalLocation ) { m_InternalLocation->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_AddressLines.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_AddressLines.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcLabel>& type_object = m_AddressLines[ii];
			if( type_object )
			{
				type_object->getStepParameter( stream, false );
			}
			else
			{
				stream << "$";
			}
		}
		stream << ")";
	}
	else { stream << "$"; }
	stream << ",";
	if( m_PostalBox ) { m_PostalBox->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Town ) { m_Town->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Region ) { m_Region->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_PostalCode ) { m_PostalCode->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Country ) { m_Country->getStepParameter( stream ); } else { stream << "$"; }
	stream << ");";
}
void IFC4X3::IfcPostalAddress::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcPostalAddress::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 10 ){ std::stringstream err; err << "Wrong parameter count for entity IfcPostalAddress, expecting 10, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Purpose = IfcAddressTypeEnum::createObjectFromSTEP( args[0], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	m_UserDefinedPurpose = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	m_InternalLocation = IfcLabel::createObjectFromSTEP( args[3], map, errorStream );
	readTypeOfStringList( args[4], m_AddressLines );
	m_PostalBox = IfcLabel::createObjectFromSTEP( args[5], map, errorStream );
	m_Town = IfcLabel::createObjectFromSTEP( args[6], map, errorStream );
	m_Region = IfcLabel::createObjectFromSTEP( args[7], map, errorStream );
	m_PostalCode = IfcLabel::createObjectFromSTEP( args[8], map, errorStream );
	m_Country = IfcLabel::createObjectFromSTEP( args[9], map, errorStream );
}
void IFC4X3::IfcPostalAddress::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcAddress::getAttributes( vec_attributes );
	vec_attributes.emplace_back( std::make_pair( "InternalLocation", m_InternalLocation ) );
	shared_ptr<AttributeObjectVector> AddressLines_vec_object( new AttributeObjectVector() );
	std::copy( m_AddressLines.begin(), m_AddressLines.end(), std::back_inserter( AddressLines_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "AddressLines", AddressLines_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "PostalBox", m_PostalBox ) );
	vec_attributes.emplace_back( std::make_pair( "Town", m_Town ) );
	vec_attributes.emplace_back( std::make_pair( "Region", m_Region ) );
	vec_attributes.emplace_back( std::make_pair( "PostalCode", m_PostalCode ) );
	vec_attributes.emplace_back( std::make_pair( "Country", m_Country ) );
}
void IFC4X3::IfcPostalAddress::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcAddress::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcPostalAddress::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcAddress::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcPostalAddress::unlinkFromInverseCounterparts()
{
	IfcAddress::unlinkFromInverseCounterparts();
}
