/**
 * @file SysPara.h
 *
 * @brief Class of system parameter
 */

#ifndef _SYS_PARA_H_
#define _SYS_PARA_H_

#include <Arduino.h>
#include <stdint.h>
#include "Storage.h"
#include "debugSerial.h"


//! system parameter data
template <typename T> struct sys_para_data
{
    T *curPtr;       //!< pointer to current value
    T min;           //!< minimum value
    T max;           //!< maximum value
};

//! system parameter class
template <class T>
class SysPara {
    public:
       /**
        * @brief Constructor
        *
        * @param curPtr    - pointer to current value
        * @param min       - minimum value
        * @param max       - maximum value
        * @param stoItemId - storage ID (optional, default=none)
        *                    providing a storage ID will enable following features:
        *                    - load initial value from storage at instantiation
        *                    - provide functions to get/set from/to storage
        */
        SysPara(T* curPtr, T min, T max, sto_item_id_t stoItemId = STO_ITEM__LAST_ENUM) {
            if (curPtr) {
                _data.curPtr = curPtr;
            } else {
                debugPrintf("%s(): empty pointer!\n", __func__);
                _data.curPtr = (T*)&_dummyCur;
            }
            _data.min = min;
            _data.max = max;
            _stoItemId = stoItemId;
            getStorage();           // init current value with storage value
        }

       /**
        * @brief Constructor
        *
        * @param sysPara - system parameter object
        */
        explicit SysPara(T& sysPara) {
            this = sysPara;
        }

       /**
        * @brief Constructor used for extern declarations
        */
        explicit SysPara() {
            _stoItemId = STO_ITEM__LAST_ENUM;
        }

       /**
        * @brief Returns the storage ID.
        *
        * @return storage ID (if exist) or STO_ITEM__LAST_ENUM
        */
        sto_item_id_t getStorageId(void) { return _stoItemId; }

       /**
        * @brief Returns the current value.
        *
        * @return current parameter value
        */
        T get(void) { return *_data.curPtr; }

       /**
        * @brief Returns the minimum value.
        *
        * @return minimum parameter value
        */
        T getMin(void) { return _data.min; }

       /**
        * @brief Returns the maximum value.
        *
        * @return maximum parameter value
        */
        T getMax(void) { return _data.max; }

       /**
        * @brief Reads the storage value into current value, if a valid storage ID
        *        was given at instantiation.
        *
        * @return  0 - success, <0 - failure
        */
        int getStorage(void) {
            int stoStatus;
            if (_stoItemId < STO_ITEM__LAST_ENUM) {                             // valid storage ID?
                // use dummy variable to comply with storage API
                if ((stoStatus = storageGet(_stoItemId, _dummyCur)) == 0) {
                    if ((_dummyCur >= _data.min) && (_dummyCur <= _data.max)) {  // did we read a valid value?
                        *_data.curPtr = _dummyCur;                               // yes -> set data to new value
                    } else {
                        *_data.curPtr = _data.min;                               // no -> set data to min value
                    }
                }
                return stoStatus;
            }
            debugPrintf("%s(): no storage ID set!\n", __func__);
            return -1;
        }

       /**
        * @brief Sets the current value.
        *
        * @param value - new current value
        *
        * @return 0 - success, <0 - failure
        */
        int set(T value) {
            if ((value >= _data.min) && (value <= _data.max)) {
                *_data.curPtr = value;
                return 0;
            }
            debugPrintf("%s(): value is outside of range!\n", __func__);
            return -1;
        }

       /**
        * @brief Writes the current value to storage, if a valid storage ID
        *        was given at instantiation.
        *
        * @param commit - true=commit storage (optional, default=false)
        *
        * @return  0 - success, <0 - failure
        */
        int setStorage(bool commit = false) {
            if (_stoItemId < STO_ITEM__LAST_ENUM) {
                if ((*_data.curPtr >= _data.min) && (*_data.curPtr <= _data.max)) {
                    return storageSet(_stoItemId, *_data.curPtr, commit);
                } else {
                    debugPrintf("%s(): value outside of allowed range! (item: %i)\n", __func__, _stoItemId);
                    return -1;
                }
            }
            debugPrintf("%s(): no storage ID set!\n", __func__);
            return -1;
        }

    private:
        sto_item_id_t _stoItemId;       //!< storage item ID (if provided at instantiation)
        sys_para_data<T> _data;         //!< parameter data
        T _dummyCur;
};


#endif
