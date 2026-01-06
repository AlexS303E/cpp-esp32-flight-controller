#pragma once
#include "json.h"
#include "domain.h"
#include <cassert>
#include <type_traits>
#include <stdexcept>

namespace json {

    class Builder;

    class BaseContext {
    protected:
        Builder& builder_;
    public:
        BaseContext(Builder& builder);
    };

    class DictItemContext;
    class ArrayItemContext;
    class KeyContext;

    class Builder {
    public:
        Builder();

        DictItemContext StartDict();
        ArrayItemContext StartArray();
        Builder& EndDict();
        Builder& EndArray();
        KeyContext Key(std::string key);
        Builder& Value(Node value);

        Node Build();

    private:
        void CanStartDict() const;
        void CanStartArray() const;
        void CanKey() const;
        void CanValue() const;
        void CanEndDict() const;
        void CanEndArray() const;

        Node root_;
        std::vector<Node*> nodes_stack_;
        std::vector<std::string> keys_;
        BuilderState state_ = BuilderState::Empty;

        friend class DictItemContext;
        friend class ArrayItemContext;
        friend class KeyContext;
    };

    class DictItemContext : public BaseContext {
    public:
        using BaseContext::BaseContext;
        KeyContext Key(std::string key);
        Builder& EndDict();
    };

    class ArrayItemContext : public BaseContext {
    public:
        using BaseContext::BaseContext;
        ArrayItemContext Value(Node value);
        DictItemContext StartDict();
        ArrayItemContext StartArray();
        Builder& EndArray();
    };

    class KeyContext : public BaseContext {
    public:
        using BaseContext::BaseContext;
        DictItemContext Value(Node value);
        ArrayItemContext StartArray();
        DictItemContext StartDict();
    };

} // namespace json