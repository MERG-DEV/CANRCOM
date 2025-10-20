# CANRCOM

PIC assembler firmware for RailCom authored by Mike Bolton

# Version 4a Build 101

This version as provided by Mike in the test1 ZIP file. There are updates as far as test6, but we have not located that source code yet.

# Description

The CANRCOM firmware runs on CANMIO or compatible hardware interfacing to a CT circuit to read channel 1 RailCom data which I transmitted on CBUS using a DDES message. 
 
Tim Calnun Pratt has designed a CANMIO daughter board, MIORCOM, which implements the CT circuit.
 
Documentation on MERG knowledgebase https://www.merg.org.uk/merg_wiki/doku.php?id=cbus:canrcom

New frontend circuit design making better use of comparators. Circuit also includes voltage converter IC. All this makes the frontend circuit more sensitive to incoming RailCom datagrams compared to the old original design.
