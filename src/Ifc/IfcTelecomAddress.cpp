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
#include "ifcpp/Ifc/IfcTelecomAddress.h"
#include "ifcpp/Ifc/IfcText.h"
#include "ifcpp/Ifc/IfcURIReference.h"

// ENTITY IfcTelecomAddress 
IFC4X3::IfcTelecomAddress::IfcTelecomAddress( int tag ) { m_tag = tag; }
void IFC4X3::IfcTelecomAddress::getStepLine( std::stringstream& stream ) const
{
	stream << "#" << m_tag << "= IFCTELECOMADDRESS" << "(";
	if( m_Purpose ) { m_Purpose->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_Description ) { m_Description->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_UserDefinedPurpose ) { m_UserDefinedPurpose->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_TelephoneNumbers.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_TelephoneNumbers.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcLabel>& type_object = m_TelephoneNumbers[ii];
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
	if( m_FacsimileNumbers.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_FacsimileNumbers.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcLabel>& type_object = m_FacsimileNumbers[ii];
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
	if( m_PagerNumber ) { m_PagerNumber->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_ElectronicMailAddresses.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_ElectronicMailAddresses.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcLabel>& type_object = m_ElectronicMailAddresses[ii];
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
	if( m_WWWHomePageURL ) { m_WWWHomePageURL->getStepParameter( stream ); } else { stream << "$"; }
	stream << ",";
	if( m_MessagingIDs.size() > 0 )
	{
		stream << "(";
		for( size_t ii = 0; ii < m_MessagingIDs.size(); ++ii )
		{
			if( ii > 0 )
			{
				stream << ",";
			}
			const shared_ptr<IfcURIReference>& type_object = m_MessagingIDs[ii];
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
	stream << ");";
}
void IFC4X3::IfcTelecomAddress::getStepParameter( std::stringstream& stream, bool /*is_select_type*/ ) const { stream << "#" << m_tag; }
void IFC4X3::IfcTelecomAddress::readStepArguments( const std::vector<std::string>& args, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	const size_t num_args = args.size();
	if( num_args != 9 ){ std::stringstream err; err << "Wrong parameter count for entity IfcTelecomAddress, expecting 9, having " << num_args << ". Entity ID: " << m_tag << std::endl; throw BuildingException( err.str().c_str() ); }
	m_Purpose = IfcAddressTypeEnum::createObjectFromSTEP( args[0], map, errorStream );
	m_Description = IfcText::createObjectFromSTEP( args[1], map, errorStream );
	m_UserDefinedPurpose = IfcLabel::createObjectFromSTEP( args[2], map, errorStream );
	readTypeOfStringList( args[3], m_TelephoneNumbers );
	readTypeOfStringList( args[4], m_FacsimileNumbers );
	m_PagerNumber = IfcLabel::createObjectFromSTEP( args[5], map, errorStream );
	readTypeOfStringList( args[6], m_ElectronicMailAddresses );
	m_WWWHomePageURL = IfcURIReference::createObjectFromSTEP( args[7], map, errorStream );
	readTypeOfStringList( args[8], m_MessagingIDs );
}
void IFC4X3::IfcTelecomAddress::getAttributes( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes ) const
{
	IFC4X3::IfcAddress::getAttributes( vec_attributes );
	shared_ptr<AttributeObjectVector> TelephoneNumbers_vec_object( new AttributeObjectVector() );
	std::copy( m_TelephoneNumbers.begin(), m_TelephoneNumbers.end(), std::back_inserter( TelephoneNumbers_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "TelephoneNumbers", TelephoneNumbers_vec_object ) );
	shared_ptr<AttributeObjectVector> FacsimileNumbers_vec_object( new AttributeObjectVector() );
	std::copy( m_FacsimileNumbers.begin(), m_FacsimileNumbers.end(), std::back_inserter( FacsimileNumbers_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "FacsimileNumbers", FacsimileNumbers_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "PagerNumber", m_PagerNumber ) );
	shared_ptr<AttributeObjectVector> ElectronicMailAddresses_vec_object( new AttributeObjectVector() );
	std::copy( m_ElectronicMailAddresses.begin(), m_ElectronicMailAddresses.end(), std::back_inserter( ElectronicMailAddresses_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "ElectronicMailAddresses", ElectronicMailAddresses_vec_object ) );
	vec_attributes.emplace_back( std::make_pair( "WWWHomePageURL", m_WWWHomePageURL ) );
	shared_ptr<AttributeObjectVector> MessagingIDs_vec_object( new AttributeObjectVector() );
	std::copy( m_MessagingIDs.begin(), m_MessagingIDs.end(), std::back_inserter( MessagingIDs_vec_object->m_vec ) );
	vec_attributes.emplace_back( std::make_pair( "MessagingIDs", MessagingIDs_vec_object ) );
}
void IFC4X3::IfcTelecomAddress::getAttributesInverse( std::vector<std::pair<std::string, shared_ptr<BuildingObject> > >& vec_attributes_inverse ) const
{
	IFC4X3::IfcAddress::getAttributesInverse( vec_attributes_inverse );
}
void IFC4X3::IfcTelecomAddress::setInverseCounterparts( shared_ptr<BuildingEntity> ptr_self_entity )
{
	IfcAddress::setInverseCounterparts( ptr_self_entity );
}
void IFC4X3::IfcTelecomAddress::unlinkFromInverseCounterparts()
{
	IfcAddress::unlinkFromInverseCounterparts();
}
