/* -*-c++-*- IfcQuery www.ifcquery.com
*
MIT License

Copyright (c) 2017 Fabian Gerold

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once

#include "BasicTypes.h"
#include "GlobalDefines.h"
#include <functional>
#include <iostream>
#include <sstream>
#include <vector>


class BuildingEntity;

class IFCQUERY_EXPORT StatusCallback {
public:
    enum MessageType {
        PROGRESS_CHANGED,
        INFO,
        WARNING,
        ERROR,
        UNKNOWN,
    };

    class Message {
    public:
        MessageType m_type = UNKNOWN;
        std::string m_text;
        const char* m_function = "";
        std::weak_ptr<BuildingEntity> m_entity;
        
        double m_progress = -1;
        std::string m_progressType;
    };

    StatusCallback() = default;
    virtual ~StatusCallback() = default;

    //\brief error callback mechanism to show messages in gui
    virtual void SetMessageCallBack( const std::function<void( const shared_ptr<Message>& )>& callback ) {
        this->m_messageCallback = callback;
    }

    virtual void SetMessageTarget( StatusCallback* other ) {
        this->m_redirectTarget = other;
    }

    virtual void SetIsCancellationRequestedMethod( const std::function<bool()>& isCancellationRequested ) {
        this->m_isCancellationRequested = isCancellationRequested;
    }

    //\brief trigger the callback to pass a message, warning, or error, for example to store in a logfile
    void SendMessage( const shared_ptr<Message>& m ) {
        if( this->m_redirectTarget ) {
            this->m_redirectTarget->SendMessage( m );
            return;
        }

        if( this->m_messageCallback ) {
            this->m_messageCallback( m );
        }
    }

    bool IsCancellationRequested() {
        if( this->m_redirectTarget ) {
            return this->m_redirectTarget->IsCancellationRequested();
        }

        if( this->m_isCancellationRequested ) {
            return this->m_isCancellationRequested();
        }

        return false;
    }

    void SendMessage( const std::string& text, MessageType type, const char* function, const std::weak_ptr<BuildingEntity>& entity = {} ) {
        auto message = std::make_shared<Message>();
        message->m_text = text;
        message->m_type = type;
        message->m_function = function;
        message->m_entity = entity;
        this->SendMessage( message );
    }
    
    void progressValueCallback( double progress, const std::string& progressType ) {
        auto message = std::make_shared<Message>();
        message->m_type = MessageType::PROGRESS_CHANGED;
        message->m_progress = progress;
        message->m_progressType = progressType;
        SendMessage( message );
    }

protected:
    std::function<void( const shared_ptr<Message>& )> m_messageCallback;
    std::function<bool()> m_isCancellationRequested;
    StatusCallback* m_redirectTarget;
};
