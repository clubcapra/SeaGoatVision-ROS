#ifdef __CPLUSPLUS__
extern "C" {
#endif

#ifndef __GNUC__
#pragma warning(disable: 4275)
#pragma warning(disable: 4101)

#endif
#include "Python.h"
#include "compile.h"
#include "frameobject.h"
#include <complex>
#include <math.h>
#include <string>
#include "scxx/object.h"
#include "scxx/list.h"
#include "scxx/tuple.h"
#include "scxx/dict.h"
#include <iostream>
#include <stdio.h>
#include "numpy/arrayobject.h"
#include "opencv2/opencv.hpp"
#include <fstream>
#include <Python.h>




// global None value for use in functions.
namespace py {
object None = object(Py_None);
}

const char* find_type(PyObject* py_obj)
{
    if(py_obj == NULL) return "C NULL value";
    if(PyCallable_Check(py_obj)) return "callable";
    if(PyString_Check(py_obj)) return "string";
    if(PyInt_Check(py_obj)) return "int";
    if(PyFloat_Check(py_obj)) return "float";
    if(PyDict_Check(py_obj)) return "dict";
    if(PyList_Check(py_obj)) return "list";
    if(PyTuple_Check(py_obj)) return "tuple";
    if(PyFile_Check(py_obj)) return "file";
    if(PyModule_Check(py_obj)) return "module";

    //should probably do more intergation (and thinking) on these.
    if(PyCallable_Check(py_obj) && PyInstance_Check(py_obj)) return "callable";
    if(PyInstance_Check(py_obj)) return "instance";
    if(PyCallable_Check(py_obj)) return "callable";
    return "unknown type";
}

void throw_error(PyObject* exc, const char* msg)
{
 //printf("setting python error: %s\n",msg);
  PyErr_SetString(exc, msg);
  //printf("throwing error\n");
  throw 1;
}

void handle_bad_type(PyObject* py_obj, const char* good_type, const char* var_name)
{
    char msg[500];
    sprintf(msg,"received '%s' type instead of '%s' for variable '%s'",
            find_type(py_obj),good_type,var_name);
    throw_error(PyExc_TypeError,msg);
}

void handle_conversion_error(PyObject* py_obj, const char* good_type, const char* var_name)
{
    char msg[500];
    sprintf(msg,"Conversion Error:, received '%s' type instead of '%s' for variable '%s'",
            find_type(py_obj),good_type,var_name);
    throw_error(PyExc_TypeError,msg);
}


class int_handler
{
public:
    int convert_to_int(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyInt_Check(py_obj))
            handle_conversion_error(py_obj,"int", name);
        return (int) PyInt_AsLong(py_obj);
    }

    int py_to_int(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyInt_Check(py_obj))
            handle_bad_type(py_obj,"int", name);
        
        return (int) PyInt_AsLong(py_obj);
    }
};

int_handler x__int_handler = int_handler();
#define convert_to_int(py_obj,name) \
        x__int_handler.convert_to_int(py_obj,name)
#define py_to_int(py_obj,name) \
        x__int_handler.py_to_int(py_obj,name)


PyObject* int_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class float_handler
{
public:
    double convert_to_float(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyFloat_Check(py_obj))
            handle_conversion_error(py_obj,"float", name);
        return PyFloat_AsDouble(py_obj);
    }

    double py_to_float(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyFloat_Check(py_obj))
            handle_bad_type(py_obj,"float", name);
        
        return PyFloat_AsDouble(py_obj);
    }
};

float_handler x__float_handler = float_handler();
#define convert_to_float(py_obj,name) \
        x__float_handler.convert_to_float(py_obj,name)
#define py_to_float(py_obj,name) \
        x__float_handler.py_to_float(py_obj,name)


PyObject* float_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class complex_handler
{
public:
    std::complex<double> convert_to_complex(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyComplex_Check(py_obj))
            handle_conversion_error(py_obj,"complex", name);
        return std::complex<double>(PyComplex_RealAsDouble(py_obj),PyComplex_ImagAsDouble(py_obj));
    }

    std::complex<double> py_to_complex(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyComplex_Check(py_obj))
            handle_bad_type(py_obj,"complex", name);
        
        return std::complex<double>(PyComplex_RealAsDouble(py_obj),PyComplex_ImagAsDouble(py_obj));
    }
};

complex_handler x__complex_handler = complex_handler();
#define convert_to_complex(py_obj,name) \
        x__complex_handler.convert_to_complex(py_obj,name)
#define py_to_complex(py_obj,name) \
        x__complex_handler.py_to_complex(py_obj,name)


PyObject* complex_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class unicode_handler
{
public:
    Py_UNICODE* convert_to_unicode(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        Py_XINCREF(py_obj);
        if (!py_obj || !PyUnicode_Check(py_obj))
            handle_conversion_error(py_obj,"unicode", name);
        return PyUnicode_AS_UNICODE(py_obj);
    }

    Py_UNICODE* py_to_unicode(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyUnicode_Check(py_obj))
            handle_bad_type(py_obj,"unicode", name);
        Py_XINCREF(py_obj);
        return PyUnicode_AS_UNICODE(py_obj);
    }
};

unicode_handler x__unicode_handler = unicode_handler();
#define convert_to_unicode(py_obj,name) \
        x__unicode_handler.convert_to_unicode(py_obj,name)
#define py_to_unicode(py_obj,name) \
        x__unicode_handler.py_to_unicode(py_obj,name)


PyObject* unicode_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class string_handler
{
public:
    std::string convert_to_string(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        Py_XINCREF(py_obj);
        if (!py_obj || !PyString_Check(py_obj))
            handle_conversion_error(py_obj,"string", name);
        return std::string(PyString_AsString(py_obj));
    }

    std::string py_to_string(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyString_Check(py_obj))
            handle_bad_type(py_obj,"string", name);
        Py_XINCREF(py_obj);
        return std::string(PyString_AsString(py_obj));
    }
};

string_handler x__string_handler = string_handler();
#define convert_to_string(py_obj,name) \
        x__string_handler.convert_to_string(py_obj,name)
#define py_to_string(py_obj,name) \
        x__string_handler.py_to_string(py_obj,name)


               PyObject* string_to_py(std::string s)
               {
                   return PyString_FromString(s.c_str());
               }
               
class list_handler
{
public:
    py::list convert_to_list(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyList_Check(py_obj))
            handle_conversion_error(py_obj,"list", name);
        return py::list(py_obj);
    }

    py::list py_to_list(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyList_Check(py_obj))
            handle_bad_type(py_obj,"list", name);
        
        return py::list(py_obj);
    }
};

list_handler x__list_handler = list_handler();
#define convert_to_list(py_obj,name) \
        x__list_handler.convert_to_list(py_obj,name)
#define py_to_list(py_obj,name) \
        x__list_handler.py_to_list(py_obj,name)


PyObject* list_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class dict_handler
{
public:
    py::dict convert_to_dict(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyDict_Check(py_obj))
            handle_conversion_error(py_obj,"dict", name);
        return py::dict(py_obj);
    }

    py::dict py_to_dict(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyDict_Check(py_obj))
            handle_bad_type(py_obj,"dict", name);
        
        return py::dict(py_obj);
    }
};

dict_handler x__dict_handler = dict_handler();
#define convert_to_dict(py_obj,name) \
        x__dict_handler.convert_to_dict(py_obj,name)
#define py_to_dict(py_obj,name) \
        x__dict_handler.py_to_dict(py_obj,name)


PyObject* dict_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class tuple_handler
{
public:
    py::tuple convert_to_tuple(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyTuple_Check(py_obj))
            handle_conversion_error(py_obj,"tuple", name);
        return py::tuple(py_obj);
    }

    py::tuple py_to_tuple(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyTuple_Check(py_obj))
            handle_bad_type(py_obj,"tuple", name);
        
        return py::tuple(py_obj);
    }
};

tuple_handler x__tuple_handler = tuple_handler();
#define convert_to_tuple(py_obj,name) \
        x__tuple_handler.convert_to_tuple(py_obj,name)
#define py_to_tuple(py_obj,name) \
        x__tuple_handler.py_to_tuple(py_obj,name)


PyObject* tuple_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class file_handler
{
public:
    FILE* convert_to_file(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        Py_XINCREF(py_obj);
        if (!py_obj || !PyFile_Check(py_obj))
            handle_conversion_error(py_obj,"file", name);
        return PyFile_AsFile(py_obj);
    }

    FILE* py_to_file(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyFile_Check(py_obj))
            handle_bad_type(py_obj,"file", name);
        Py_XINCREF(py_obj);
        return PyFile_AsFile(py_obj);
    }
};

file_handler x__file_handler = file_handler();
#define convert_to_file(py_obj,name) \
        x__file_handler.convert_to_file(py_obj,name)
#define py_to_file(py_obj,name) \
        x__file_handler.py_to_file(py_obj,name)


               PyObject* file_to_py(FILE* file, const char* name,
                                    const char* mode)
               {
                   return (PyObject*) PyFile_FromFile(file,
                     const_cast<char*>(name),
                     const_cast<char*>(mode), fclose);
               }
               
class instance_handler
{
public:
    py::object convert_to_instance(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !PyInstance_Check(py_obj))
            handle_conversion_error(py_obj,"instance", name);
        return py::object(py_obj);
    }

    py::object py_to_instance(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyInstance_Check(py_obj))
            handle_bad_type(py_obj,"instance", name);
        
        return py::object(py_obj);
    }
};

instance_handler x__instance_handler = instance_handler();
#define convert_to_instance(py_obj,name) \
        x__instance_handler.convert_to_instance(py_obj,name)
#define py_to_instance(py_obj,name) \
        x__instance_handler.py_to_instance(py_obj,name)


PyObject* instance_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class numpy_size_handler
{
public:
    void conversion_numpy_check_size(PyArrayObject* arr_obj, int Ndims,
                                     const char* name)
    {
        if (arr_obj->nd != Ndims)
        {
            char msg[500];
            sprintf(msg,"Conversion Error: received '%d' dimensional array instead of '%d' dimensional array for variable '%s'",
                    arr_obj->nd,Ndims,name);
            throw_error(PyExc_TypeError,msg);
        }
    }

    void numpy_check_size(PyArrayObject* arr_obj, int Ndims, const char* name)
    {
        if (arr_obj->nd != Ndims)
        {
            char msg[500];
            sprintf(msg,"received '%d' dimensional array instead of '%d' dimensional array for variable '%s'",
                    arr_obj->nd,Ndims,name);
            throw_error(PyExc_TypeError,msg);
        }
    }
};

numpy_size_handler x__numpy_size_handler = numpy_size_handler();
#define conversion_numpy_check_size x__numpy_size_handler.conversion_numpy_check_size
#define numpy_check_size x__numpy_size_handler.numpy_check_size


class numpy_type_handler
{
public:
    void conversion_numpy_check_type(PyArrayObject* arr_obj, int numeric_type,
                                     const char* name)
    {
        // Make sure input has correct numeric type.
        int arr_type = arr_obj->descr->type_num;
        if (PyTypeNum_ISEXTENDED(numeric_type))
        {
        char msg[80];
        sprintf(msg, "Conversion Error: extended types not supported for variable '%s'",
                name);
        throw_error(PyExc_TypeError, msg);
        }
        if (!PyArray_EquivTypenums(arr_type, numeric_type))
        {

        const char* type_names[23] = {"bool", "byte", "ubyte","short", "ushort",
                                "int", "uint", "long", "ulong", "longlong", "ulonglong",
                                "float", "double", "longdouble", "cfloat", "cdouble",
                                "clongdouble", "object", "string", "unicode", "void", "ntype",
                                "unknown"};
        char msg[500];
        sprintf(msg,"Conversion Error: received '%s' typed array instead of '%s' typed array for variable '%s'",
                type_names[arr_type],type_names[numeric_type],name);
        throw_error(PyExc_TypeError,msg);
        }
    }

    void numpy_check_type(PyArrayObject* arr_obj, int numeric_type, const char* name)
    {
        // Make sure input has correct numeric type.
        int arr_type = arr_obj->descr->type_num;
        if (PyTypeNum_ISEXTENDED(numeric_type))
        {
        char msg[80];
        sprintf(msg, "Conversion Error: extended types not supported for variable '%s'",
                name);
        throw_error(PyExc_TypeError, msg);
        }
        if (!PyArray_EquivTypenums(arr_type, numeric_type))
        {
            const char* type_names[23] = {"bool", "byte", "ubyte","short", "ushort",
                                    "int", "uint", "long", "ulong", "longlong", "ulonglong",
                                    "float", "double", "longdouble", "cfloat", "cdouble",
                                    "clongdouble", "object", "string", "unicode", "void", "ntype",
                                    "unknown"};
            char msg[500];
            sprintf(msg,"received '%s' typed array instead of '%s' typed array for variable '%s'",
                    type_names[arr_type],type_names[numeric_type],name);
            throw_error(PyExc_TypeError,msg);
        }
    }
};

numpy_type_handler x__numpy_type_handler = numpy_type_handler();
#define conversion_numpy_check_type x__numpy_type_handler.conversion_numpy_check_type
#define numpy_check_type x__numpy_type_handler.numpy_check_type


class numpy_handler
{
public:
    PyArrayObject* convert_to_numpy(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        Py_XINCREF(py_obj);
        if (!py_obj || !PyArray_Check(py_obj))
            handle_conversion_error(py_obj,"numpy", name);
        return (PyArrayObject*) py_obj;
    }

    PyArrayObject* py_to_numpy(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !PyArray_Check(py_obj))
            handle_bad_type(py_obj,"numpy", name);
        Py_XINCREF(py_obj);
        return (PyArrayObject*) py_obj;
    }
};

numpy_handler x__numpy_handler = numpy_handler();
#define convert_to_numpy(py_obj,name) \
        x__numpy_handler.convert_to_numpy(py_obj,name)
#define py_to_numpy(py_obj,name) \
        x__numpy_handler.py_to_numpy(py_obj,name)


PyObject* numpy_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


class catchall_handler
{
public:
    py::object convert_to_catchall(PyObject* py_obj, const char* name)
    {
        // Incref occurs even if conversion fails so that
        // the decref in cleanup_code has a matching incref.
        
        if (!py_obj || !(py_obj))
            handle_conversion_error(py_obj,"catchall", name);
        return py::object(py_obj);
    }

    py::object py_to_catchall(PyObject* py_obj, const char* name)
    {
        // !! Pretty sure INCREF should only be called on success since
        // !! py_to_xxx is used by the user -- not the code generator.
        if (!py_obj || !(py_obj))
            handle_bad_type(py_obj,"catchall", name);
        
        return py::object(py_obj);
    }
};

catchall_handler x__catchall_handler = catchall_handler();
#define convert_to_catchall(py_obj,name) \
        x__catchall_handler.convert_to_catchall(py_obj,name)
#define py_to_catchall(py_obj,name) \
        x__catchall_handler.py_to_catchall(py_obj,name)


PyObject* catchall_to_py(PyObject* obj)
{
    return (PyObject*) obj;
}


            py::dict params;
            py::object py_init_param;
            void init_param(char* name, int min, int max, int def_val) {
                    py::tuple args(4);
                    args[0] = name;
                    args[1] = min;
                    args[2] = max;
                    args[3] = def_val;
                    py_init_param.call(args);            
            } 
            long ParameterAsInt(char* name, int min, int max, int def_val) {
                if(!params.has_key(name)) {
                    init_param(name, min, max, def_val);
                }
                py::object o = params.get(name);
                return PyInt_AsLong(o.mcall("get_current_value"));
            }
            
            bool ParameterAsBool(char* name, int min, int max, int def_val) {
                if(!params.has_key(name)) {
                    init_param(name, min, max, def_val);
                }
                return PyInt_AsLong(params.get(name).mcall("get_current_value"));
            }            
        #include "opencv2/opencv.hpp"
//#include "opencv2/gpu/gpu.hpp"
#include <fstream>
#include <stdio.h>

#define DOCSTRING "C++ Warp Perspective"

using namespace cv;

int rotationX_ = -160;
int rotationY_ = 0;
int zoom_ = 100;

void init() {
	rotationX_ = ParameterAsInt("rotationX_", -360, 360, -160);
	rotationY_ = ParameterAsInt("rotationY_", -360, 360, 0);
	zoom_ = ParameterAsInt("zoom_", 0, 1000, 100);
}

void configure() {
	init();
}

cv::Mat execute(cv::Mat image)
{
	//Load parameters from file
	//std::ifstream infile("test.txt");
	//infile >> rotationX_ >> rotationY_ >> zoom_;

	const double w = image.cols;
	const double h = image.rows;

	// Projection 2D -> 3D matrix
	Mat A1 = (Mat_<double>(4,3) <<
		1, 0, -w/2,
		0, 1, -h/2,
		0, 0,    0,
		0, 0,    1);

	// Rotation matrices around the X axis
	const double alpha = rotationX_ / 180. * 3.1416 / w;
	Mat RX = (Mat_<double>(4, 4) <<
		1,          0,           0, 0,
		0, cos(alpha), -sin(alpha), 0,
		0, sin(alpha),  cos(alpha), 0,
		0,          0,           0, 1);

	// Rotation matrices around the X axis

	const double beta = rotationY_ / 180. * 3.1416 / h;
	Mat RY = (Mat_<double>(4, 4) <<
		cos(beta),  0,           sin(beta), 0,
		0,			1,			 0, 0,
		-sin(beta), 0,			 cos(beta), 0,
		0,          0,           0, 1);

	// Translation matrix on the Z axis
	const double dist = zoom_ / 100.0;
	Mat T = (Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, dist,
		0, 0, 0, 1);

	const double f = 1;
	// Camera Intrisecs matrix 3D -> 2D
	Mat A2 = (Mat_<double>(3,4) <<
		f, 0, w/2, 0,
		0, f, h/2, 0,
		0, 0,   1, 0);

	Mat transfo = A2 * (T * (RX * RY * A1));

	Mat destination;

	warpPerspective(image, destination, transfo, image.size(), INTER_CUBIC | WARP_INVERSE_MAP);

	return destination;
}


static PyObject* exec_PerspectiveByAngle(PyObject*self, PyObject* args, PyObject* kywds)
{
    py::object return_val;
    int exception_occurred = 0;
    PyObject *py_local_dict = NULL;
    static const char *kwlist[] = {"image","local_dict", NULL};
    PyObject *py_image;
    int image_used;
    py_image = NULL;
    image_used = 0;
    
    if(!PyArg_ParseTupleAndKeywords(args,kywds,"O|O:exec_PerspectiveByAngle",const_cast<char**>(kwlist),&py_image, &py_local_dict))
       return NULL;
    try                              
    {                                
        py_image = py_image;
        PyArrayObject* image_array = convert_to_numpy(py_image,"image");
        conversion_numpy_check_type(image_array,PyArray_UBYTE,"image");
        #define IMAGE1(i) (*((npy_ubyte*)(image_array->data + (i)*Simage[0])))
        #define IMAGE2(i,j) (*((npy_ubyte*)(image_array->data + (i)*Simage[0] + (j)*Simage[1])))
        #define IMAGE3(i,j,k) (*((npy_ubyte*)(image_array->data + (i)*Simage[0] + (j)*Simage[1] + (k)*Simage[2])))
        #define IMAGE4(i,j,k,l) (*((npy_ubyte*)(image_array->data + (i)*Simage[0] + (j)*Simage[1] + (k)*Simage[2] + (l)*Simage[3])))
        npy_intp* Nimage = image_array->dimensions;
        npy_intp* Simage = image_array->strides;
        int Dimage = image_array->nd;
        npy_ubyte* image = (npy_ubyte*) image_array->data;
        image_used = 1;
        /*<function call here>*/     
        
                    cv::Mat mat(Nimage[0], Nimage[1], CV_8UC(3), image);
                    cv::Mat ret = execute(mat);
                    if (mat.data != ret.data) {
                        ret.copyTo(mat);
                    }
        
        if(py_local_dict)                                  
        {                                                  
            py::dict local_dict = py::dict(py_local_dict); 
        }                                                  
    
    }                                
    catch(...)                       
    {                                
        return_val =  py::object();      
        exception_occurred = 1;       
    }                                
    /*cleanup code*/                     
    if(image_used)
    {
        Py_XDECREF(py_image);
        #undef IMAGE1
        #undef IMAGE2
        #undef IMAGE3
        #undef IMAGE4
    }
    if(!(PyObject*)return_val && !exception_occurred)
    {
                                  
        return_val = Py_None;            
    }
                                  
    return return_val.disown();           
}                                
static PyObject* help_PerspectiveByAngle(PyObject*self, PyObject* args, PyObject* kywds)
{
    py::object return_val;
    int exception_occurred = 0;
    PyObject *py_local_dict = NULL;
    static const char *kwlist[] = {"local_dict", NULL};
    if(!PyArg_ParseTupleAndKeywords(args,kywds,"|O:help_PerspectiveByAngle",const_cast<char**>(kwlist),&py_local_dict))
       return NULL;
    try                              
    {                                
        /*<function call here>*/     
        
                    #ifdef DOCSTRING
                    return_val = DOCSTRING;
                    #else
                    return_val = "";
                    #endif
        if(py_local_dict)                                  
        {                                                  
            py::dict local_dict = py::dict(py_local_dict); 
        }                                                  
    
    }                                
    catch(...)                       
    {                                
        return_val =  py::object();      
        exception_occurred = 1;       
    }                                
    /*cleanup code*/                     
    if(!(PyObject*)return_val && !exception_occurred)
    {
                                  
        return_val = Py_None;            
    }
                                  
    return return_val.disown();           
}                                
static PyObject* init_PerspectiveByAngle(PyObject*self, PyObject* args, PyObject* kywds)
{
    py::object return_val;
    int exception_occurred = 0;
    PyObject *py_local_dict = NULL;
    static const char *kwlist[] = {"p","pip","local_dict", NULL};
    PyObject *py_p, *py_pip;
    int p_used, pip_used;
    py_p = py_pip = NULL;
    p_used= pip_used = 0;
    
    if(!PyArg_ParseTupleAndKeywords(args,kywds,"OO|O:init_PerspectiveByAngle",const_cast<char**>(kwlist),&py_p, &py_pip, &py_local_dict))
       return NULL;
    try                              
    {                                
        py_p = py_p;
        py::dict p = convert_to_dict(py_p,"p");
        p_used = 1;
        py_pip = py_pip;
        py::object pip = convert_to_catchall(py_pip,"pip");
        pip_used = 1;
        /*<function call here>*/     
        
                params = p;
                py_init_param = pip;
                init();
        if(py_local_dict)                                  
        {                                                  
            py::dict local_dict = py::dict(py_local_dict); 
        }                                                  
    
    }                                
    catch(...)                       
    {                                
        return_val =  py::object();      
        exception_occurred = 1;       
    }                                
    /*cleanup code*/                     
    if(!(PyObject*)return_val && !exception_occurred)
    {
                                  
        return_val = Py_None;            
    }
                                  
    return return_val.disown();           
}                                
static PyObject* config_PerspectiveByAngle(PyObject*self, PyObject* args, PyObject* kywds)
{
    py::object return_val;
    int exception_occurred = 0;
    PyObject *py_local_dict = NULL;
    static const char *kwlist[] = {"local_dict", NULL};
    if(!PyArg_ParseTupleAndKeywords(args,kywds,"|O:config_PerspectiveByAngle",const_cast<char**>(kwlist),&py_local_dict))
       return NULL;
    try                              
    {                                
        /*<function call here>*/     
        
                    configure();
        if(py_local_dict)                                  
        {                                                  
            py::dict local_dict = py::dict(py_local_dict); 
        }                                                  
    
    }                                
    catch(...)                       
    {                                
        return_val =  py::object();      
        exception_occurred = 1;       
    }                                
    /*cleanup code*/                     
    if(!(PyObject*)return_val && !exception_occurred)
    {
                                  
        return_val = Py_None;            
    }
                                  
    return return_val.disown();           
}                                


static PyMethodDef compiled_methods[] = 
{
    {"exec_PerspectiveByAngle",(PyCFunction)exec_PerspectiveByAngle , METH_VARARGS|METH_KEYWORDS},
    {"help_PerspectiveByAngle",(PyCFunction)help_PerspectiveByAngle , METH_VARARGS|METH_KEYWORDS},
    {"init_PerspectiveByAngle",(PyCFunction)init_PerspectiveByAngle , METH_VARARGS|METH_KEYWORDS},
    {"config_PerspectiveByAngle",(PyCFunction)config_PerspectiveByAngle , METH_VARARGS|METH_KEYWORDS},
    {NULL,      NULL}        /* Sentinel */
};

PyMODINIT_FUNC initPerspectiveByAngle(void)
{
    
    Py_Initialize();
    import_array();
    PyImport_ImportModule("numpy");
    (void) Py_InitModule("PerspectiveByAngle", compiled_methods);
}

#ifdef __CPLUSCPLUS__
}
#endif
