# Included only for BARCODE_LIVE_ENABLE=1: the V1 (pending expiry) and V3
# (SEARCH cancel) policy. Forced includes on vehicle_task.o and
# junction_print.o route ja_pending_route/ja_reason_name through
# app_program/barcode_live_vehicle/, which every other profile omits.
mtkernel_3/app_program/vehicle/vehicle_task.o: CFLAGS += -include ../app_program/barcode_live_vehicle/vehicle_impl.h
mtkernel_3/app_program/vehicle/junction_print.o: CFLAGS += -include ../app_program/barcode_live_vehicle/print_impl.h
