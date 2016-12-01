#pragma once

#include <string>
#include <sstream>
#include <math/math.hpp>
#include <boost/lexical_cast.hpp>


namespace boost
{

    namespace detail
    {

        template<class Type>
        struct base_conversion
        {
            static std::string save_to_string(const Type &arg)
            {
                std::ostringstream str;

                for
                (
                    typename Type::const_iterator begin = arg.begin(), it = begin, end = arg.end();
                    it != end;
                    ++it
                )
                {
                    if (it != begin) str << ' ';
                    str << *it;
                }

                return str.str();
            }
        };

        template<class Type, typename InternalType = typename Type::value_type>
        struct conversion
        {
            static Type load_from_cstr(char *start)
            {
                throw_exception(bad_lexical_cast(typeid(std::string), typeid(Type)));
            }
        };

        template<class Type>
        struct conversion<Type, float> : public base_conversion<Type>
        {
            static Type load_from_cstr(char *start)
            {
                Type r;

                char *stop  = start;

                for
                (
                    typename Type::iterator it = r.begin(), end = r.end();
                    it != end;
                    ++it
                )
                {
                    *it = typename Type::value_type(strtod(start, &stop));
                    if (start != stop)
                        start = stop;
                    else
                        throw_exception(bad_lexical_cast(typeid(std::string), typeid(Type)));
                }

                return r;
            }
        };

        template<class Type>
        struct conversion<Type, double> : public base_conversion<Type>
        {
            static Type load_from_cstr(char *str)
            {
                Type r;

                char *start = str;
                char *stop  = start;

                for
                (
                    typename Type::iterator it = r.begin(), end = r.end();
                    it != end;
                    ++it
                )
                {
                    *it = typename Type::value_type(strtod(start, &stop));
                    if (start != stop)
                        start = stop;
                    else
                        throw_exception(bad_lexical_cast(typeid(std::string), typeid(Type)));
                }

                return r;
            }
        };

        template<class Type>
        struct conversion<Type, int> : public base_conversion<Type>
        {
            static Type load_from_cstr(char *str)
            {
                Type r;

                char *start = str;
                char *stop  = start;

                for
                (
                    typename Type::iterator it = r.begin(), end = r.end();
                    it != end;
                    ++it
                )
                {
                    *it = typename Type::value_type(strtol(start, &stop, 10));
                    if (start != stop)
                        start = stop;
                    else
                        throw bad_lexical_cast(typeid(std::string), typeid(Type));
                }

                return r;
            }
        };

        template<class Type>
        struct conversion<Type, long> : public base_conversion<Type>
        {
            static Type load_from_cstr(char *str)
            {
                Type r;

                char *start = str;
                char *stop  = start;

                for
                (
                    typename Type::iterator it = r.begin(), end = r.end();
                    it != end;
                    ++it
                )
                {
                    *it = typename Type::value_type(strtol(start, &stop, 10));
                    if (start != stop)
                        start = stop;
                    else
                        throw_exception(bad_lexical_cast(typeid(std::string), typeid(Type)));
                }

                return r;
            }
        };

    }


    template<>
    inline nMath::Vector2f lexical_cast<nMath::Vector2f, std::string>(const std::string &str)
    { return detail::conversion<nMath::Vector2f>::load_from_cstr(const_cast<char*>(str.c_str())); }

    template<>
    inline std::string lexical_cast<std::string, nMath::Vector2f>(const nMath::Vector2f &v)
    { return detail::conversion<nMath::Vector2f>::save_to_string(v); }


    template<>
    inline nMath::Vector3f lexical_cast<nMath::Vector3f, std::string>(const std::string &str)
    { return detail::conversion<nMath::Vector3f>::load_from_cstr(const_cast<char*>(str.c_str())); }

    template<>
    inline std::string lexical_cast<std::string, nMath::Vector3f>(const nMath::Vector3f &v)
    { return detail::conversion<nMath::Vector3f>::save_to_string(v); }


    template<>
    inline nMath::Vector4f lexical_cast<nMath::Vector4f, std::string>(const std::string &str)
    { return detail::conversion<nMath::Vector4f>::load_from_cstr(const_cast<char*>(str.c_str())); }

    template<>
    inline std::string lexical_cast<std::string, nMath::Vector4f>(const nMath::Vector4f &v)
    { return detail::conversion<nMath::Vector4f>::save_to_string(v); }


    template<>
    inline nMath::Quatf lexical_cast<nMath::Quatf, std::string>(const std::string &str)
    { return detail::conversion<nMath::Quatf>::load_from_cstr(const_cast<char*>(str.c_str())); }

    template<>
    inline std::string lexical_cast<std::string, nMath::Quatf>(const nMath::Quatf &v)
    { return detail::conversion<nMath::Quatf>::save_to_string(v); }

}
