/*
 */
#include <vector>
#include <algorithm>
#include <stdexcept>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// https://medium.com/@ahmedfgad/handling-python-numpy-arrays-in-c-using-pybind11-0b7450f4f4b3
#include <pybind11/numpy.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

#define SEPARATOR_0 0
#define SEPARATOR_1 1
#define SEPARATOR_2 2
#define STATUS_ERRORS_MSB 3
#define STATUS_ERRORS_LSB 4
#define STATUS_CRC 5
#define SEPARATOR_SIZE 6

class Decoder
{
private:
    uint8_t crc = 0xFF;
    uint16_t errors = 0xFFFF;
    std::vector<unsigned char> buffer;

    // Finds the next separator and returns its index
    // return 0: no separator found
    inline int32_t find_first_separator() const
    {
        int32_t last = buffer.size() - SEPARATOR_SIZE + 1;
        for (int32_t i = 0; i < last; i += 3)
        {
            if (true                                  //
                && (buffer[i + SEPARATOR_0] == 0x7F)  //
                && (buffer[i + SEPARATOR_1] == 0xFF)  //
                && (buffer[i + SEPARATOR_2] == 0xFF)) //
            {
                return i;
            }
            continue;
        }
        return 0; // No separator
    }

    inline int8_t calculate_crc(int32_t index_first_separator) const
    {
        int8_t crc = 0;
        for (int32_t i = 0; i < index_first_separator + SEPARATOR_SIZE; i++)
        {
            crc ^= buffer[i];
        }
        return crc;
    }

public:
    void clear()
    {
        buffer.clear();
    }

    // If a separator is found:
    // Purges all measurements up to and including the separator
    // returns the bytes count purged.
    //
    // return 0: no separator found
    inline int32_t purge_until_and_with_separator()
    {
        int32_t last = buffer.size() - SEPARATOR_SIZE + 1;
        for (int32_t i = 0; i < last; i++)
        {
            if (true                                  //
                && (buffer[i + SEPARATOR_0] == 0x7F)  //
                && (buffer[i + SEPARATOR_1] == 0xFF)  //
                && (buffer[i + SEPARATOR_2] == 0xFF)) //
            {
                buffer.erase(buffer.begin(), buffer.begin() + i + SEPARATOR_SIZE);

                return i + SEPARATOR_SIZE;
            }
            continue;
        }
        return 0; // No separator
    }

    void push_bytes(std::string buf)
    {
        buffer.insert(buffer.end(), buf.begin(), buf.end());
        crc = 0xFF;
        errors = 0xFFFF;
    }

    // Returns: pybind11::array_t<int32_t>
    py::object get_numpy_array()
    {
        int32_t index_first_separator = find_first_separator();
        if (index_first_separator == 0)
        {
            crc = 0xFF;
            errors = 0xFFFF;
            // Need more data
            return py::none();
        }

        crc = calculate_crc(index_first_separator);
        uint8_t errors_msb = buffer[index_first_separator + STATUS_ERRORS_MSB];
        uint8_t errors_lsb = buffer[index_first_separator + STATUS_ERRORS_LSB];
        errors = (errors_msb << 8) | errors_lsb;

        if ((index_first_separator % 3) != 0)
        {
            printf("WARNING: index_first_separator=%d which is not a multiple of 3!\n", index_first_separator);
        }
        size_t measurement_count = index_first_separator / 3;
        py::array_t<int32_t> narray(measurement_count);

        // Obtain mutable access to the array
        auto narray_uncheckec = narray.mutable_unchecked<1>();

        // Populate the array with values
        for (size_t i = 0; i < measurement_count; i++)
        {
            int32_t measurement_value = 0;
            for (size_t j = 0; j < 3; j++)
            {
                measurement_value <<= 8;
                measurement_value += buffer[3 * i + j];
            }
            if (measurement_value & 0x800000)
            {
                measurement_value -= 0x1000000;
            }

            narray_uncheckec(i) = static_cast<int32_t>(measurement_value);
        }

        buffer.erase(buffer.begin(), buffer.begin() + index_first_separator + SEPARATOR_SIZE);

        return narray;
    }

    size_t size() const
    {
        return buffer.size();
    }

    py::bytes get_buffer()
    {
        return py::bytes(reinterpret_cast<const char *>(buffer.data()), buffer.size());
    }
    uint8_t get_crc()
    {
        return crc;
    }
    uint16_t get_errors()
    {
        return errors;
    }
};

PYBIND11_MODULE(ad_low_noise_float_2023_decoder, m)
{
    py::class_<Decoder>(m, "Decoder")
        .def(py::init<>())
        .def("push_bytes", &Decoder::push_bytes, py::arg("buf"),
             "Adds elements to the begin of the buffer.")
        .def("get_numpy_array", &Decoder::get_numpy_array,
             "Return numarray.")
        .def("clear", &Decoder::clear,
             "Clears the buffer.")
        .def("purge_until_and_with_separator", &Decoder::purge_until_and_with_separator,
             "Purges all measurement until the first separator. Returns the bytes purged.")
        .def("size", &Decoder::size,
             "Returns the current size of the buffer.")
        .def("get_crc", &Decoder::get_crc,
             "get_crc.")
        .def("get_errors", &Decoder::get_errors,
             "get_errors.")
        .def("get_buffer", &Decoder::get_buffer,
             "Returns the buffer.");

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
