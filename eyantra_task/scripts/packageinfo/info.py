#!/usr/bin/env python
"""
This module contain different functions which can be uses to get the information 
about package like their Cost,Stock keeping unit,Priority and Storage Info ,etc 
on the basis of their colour and descriptions.
"""
import datetime

#-----------------------------Package Priority------------------------------
def package_priority(arg_package_color):
    """
    This function return the priority of the package on the basis of their color
    :param arg_package_color: colour of the package
    :type arg_package_color: str
    :return: priority
    """
    if arg_package_color == "red":
        priority = "HP"

    elif arg_package_color == "yellow":
        priority = "MP"

    elif arg_package_color == "green":
        priority = "LP"
 
    else:
        priority = "NA"

    return priority

#-------------------------------Package Item-----------------------------------
def package_item(arg_package_color):
    """
    This function return the item description of the package on the basis of their color
    :param arg_package_color: colour of the package
    :type arg_package_color: str
    :return: item
    """
    if arg_package_color == "red":
        item = "Medicine"

    elif arg_package_color == "yellow":
        item = "Food"

    elif arg_package_color == "green":
        item = "Clothes"
 
    else:
        item = "NA"

    return item

#------------------------------Package Cost-------------------------------------
def package_cost(arg_package_color):
    """
    This function return the cost of the package on the basis of their color
    :param arg_package_color: colour of the package
    :type arg_package_color: str
    :return: cost
    """
    if arg_package_color == "red":
        cost = 450

    elif arg_package_color == "yellow":
        cost = 250

    elif arg_package_color == "green":
        cost = 150
 
    else:
        cost = "NA"

    return cost

#-----------------------------Storage Number-------------------------------------
def storage_number(arg_row , arg_column):
    """
    This Function return the Storage Number of a package on the shelf
    :param arg_row: row in which package is present
    :type arg_row: int
    :param arg_column: column in which package is presnt
    :type arg_column: int
    :return: Storage Number
    """
    storage_number = "R" + str(arg_row) + " "  + "C" + str(arg_column)

    return storage_number

#----------------------------Stocking Keeping Unit-------------------------------
def Stock_Keeping_Unit(arg_row , arg_column, arg_package_color):
    """
    This Function return the Stock Keeping Unit information of a package 
    :param arg_row: row in which package is present
    :type arg_row: int
    :param arg_column: column in which package is presnt
    :type arg_column: int
    :param arg_package_color: colour of the package
    :type arg_package_color: str
    :return: Stocking Keeping Unit
    """
    current_date = datetime.datetime.now()
    month = str(current_date.strftime("%m"))
    year =  str(current_date.strftime("%y"))

    if arg_package_color == "red" :
        SKU = "R" + str(arg_row) + str(arg_column) + month + year

    elif arg_package_color == "green" :
        SKU = "G" + str(arg_row) + str(arg_column) + month + year
          
    elif arg_package_color == "yellow" :
        SKU = "Y" + str(arg_row) + str(arg_column) + month + year
        
    else:
        SKU = "NA"

    return SKU

#------------------------------Order Cost-------------------------------------------
def order_cost(arg_item):
    """
    This Function return the cost of an order on the basis of its description
    :param arg_item: description of the order
    :type arg_item: str
    :return: cost
    """
    if arg_item == "Medicine":
        cost = 450

    elif arg_item == "Food":
        cost = 250

    elif arg_item == "Clothes":
        cost = 150

    else:
        cost = "NA"

    return cost

#--------------------------------Order Priority------------------------------------------
def order_priority(arg_item):
    """
    This Function return the priority of an order on the basis of its description
    :param arg_item: description of the order
    :type arg_item: str
    :return: cost
    """
    if arg_item == "Medicine":
        priority = "HP"

    elif arg_item == "Food":
        priority = "MP"

    elif arg_item == "Clothes":
        priority = "LP"

    else:
        priority = "NA"

    return priority

                     






