#ifndef PRINTER_RECEIVE_H
#define PRINTER_RECEIVE_H

#pragma once

void printer_receive_start(void *printer_handle);

void printer_receive_print_complete(void *printer_handle);

#endif // PRINTER_RECEIVE_H
