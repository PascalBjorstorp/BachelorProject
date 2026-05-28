; ModuleID = '/home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria/MPC_FPGA/mpc_fpga_top_opencl/hls/.autopilot/db/a.g.ld.5.gdce.bc'
source_filename = "llvm-link"
target datalayout = "e-m:e-i64:64-i128:128-i256:256-i512:512-i1024:1024-i2048:2048-i4096:4096-n8:16:32:64-S128-v16:16-v24:32-v32:32-v48:64-v96:128-v192:256-v256:256-v512:512-v1024:1024"
target triple = "fpga64-xilinx-none"

%"struct.ap_uint<512>" = type { %"struct.ap_int_base<512, false>" }
%"struct.ap_int_base<512, false>" = type { %"struct.ssdm_int<512, false>" }
%"struct.ssdm_int<512, false>" = type { i512 }
%"struct.ap_uint<128>" = type { %"struct.ap_int_base<128, false>" }
%"struct.ap_int_base<128, false>" = type { %"struct.ssdm_int<128, false>" }
%"struct.ssdm_int<128, false>" = type { i128 }

; Function Attrs: noinline willreturn
define void @apatb_mpc_fpga_top_opencl_ir(%"struct.ap_uint<512>"* noalias nonnull readonly "maxi" %input_words512, %"struct.ap_uint<128>"* noalias nonnull "maxi" %output_words128) local_unnamed_addr #0 {
entry:
  %0 = bitcast %"struct.ap_uint<512>"* %input_words512 to [11 x %"struct.ap_uint<512>"]*
  %input_words512_copy = alloca [11 x i512], align 512
  %1 = bitcast %"struct.ap_uint<128>"* %output_words128 to [1 x %"struct.ap_uint<128>"]*
  %output_words128_copy = alloca [1 x i128], align 512
  %2 = getelementptr [1 x i128], [1 x i128]* %output_words128_copy, i64 0, i64 0
  call fastcc void @copy_in([11 x %"struct.ap_uint<512>"]* nonnull %0, [11 x i512]* nonnull align 512 %input_words512_copy, [1 x %"struct.ap_uint<128>"]* nonnull %1, [1 x i128]* nonnull align 512 %output_words128_copy)
  call void @apatb_mpc_fpga_top_opencl_hw([11 x i512]* %input_words512_copy, i128* %2)
  call void @copy_back([11 x %"struct.ap_uint<512>"]* %0, [11 x i512]* %input_words512_copy, [1 x %"struct.ap_uint<128>"]* %1, [1 x i128]* %output_words128_copy)
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @copy_in([11 x %"struct.ap_uint<512>"]* noalias readonly "unpacked"="0", [11 x i512]* noalias nocapture align 512 "unpacked"="1.0", [1 x %"struct.ap_uint<128>"]* noalias readonly "unpacked"="2", [1 x i128]* noalias nocapture align 512 "unpacked"="3.0") unnamed_addr #1 {
entry:
  call fastcc void @"onebyonecpy_hls.p0a11struct.ap_uint<512>.1100"([11 x i512]* align 512 %1, [11 x %"struct.ap_uint<512>"]* %0)
  call fastcc void @"onebyonecpy_hls.p0a1struct.ap_uint<128>"([1 x i128]* align 512 %3, [1 x %"struct.ap_uint<128>"]* %2)
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @"onebyonecpy_hls.p0a11struct.ap_uint<512>"([11 x %"struct.ap_uint<512>"]* noalias "unpacked"="0" %dst, [11 x i512]* noalias nocapture readonly align 512 "unpacked"="1.0" %src) unnamed_addr #2 {
entry:
  %0 = icmp eq [11 x %"struct.ap_uint<512>"]* %dst, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  call void @"arraycpy_hls.p0a11struct.ap_uint<512>"([11 x %"struct.ap_uint<512>"]* nonnull %dst, [11 x i512]* %src, i64 11)
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define void @"arraycpy_hls.p0a11struct.ap_uint<512>"([11 x %"struct.ap_uint<512>"]* "unpacked"="0" %dst, [11 x i512]* nocapture readonly "unpacked"="1.0" %src, i64 "unpacked"="2" %num) local_unnamed_addr #3 {
entry:
  %0 = icmp eq [11 x %"struct.ap_uint<512>"]* %dst, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  %for.loop.cond1 = icmp sgt i64 %num, 0
  br i1 %for.loop.cond1, label %for.loop.lr.ph, label %copy.split

for.loop.lr.ph:                                   ; preds = %copy
  br label %for.loop

for.loop:                                         ; preds = %for.loop, %for.loop.lr.ph
  %for.loop.idx2 = phi i64 [ 0, %for.loop.lr.ph ], [ %for.loop.idx.next, %for.loop ]
  %src.addr.0.0.05 = getelementptr [11 x i512], [11 x i512]* %src, i64 0, i64 %for.loop.idx2
  %dst.addr.0.0.06 = getelementptr [11 x %"struct.ap_uint<512>"], [11 x %"struct.ap_uint<512>"]* %dst, i64 0, i64 %for.loop.idx2, i32 0, i32 0, i32 0
  %1 = load i512, i512* %src.addr.0.0.05, align 64
  store i512 %1, i512* %dst.addr.0.0.06, align 64
  %for.loop.idx.next = add nuw nsw i64 %for.loop.idx2, 1
  %exitcond = icmp ne i64 %for.loop.idx.next, %num
  br i1 %exitcond, label %for.loop, label %copy.split

copy.split:                                       ; preds = %for.loop, %copy
  br label %ret

ret:                                              ; preds = %copy.split, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @"onebyonecpy_hls.p0a1struct.ap_uint<128>"([1 x i128]* noalias nocapture align 512 "unpacked"="0.0" %dst, [1 x %"struct.ap_uint<128>"]* noalias readonly "unpacked"="1" %src) unnamed_addr #2 {
entry:
  %0 = icmp eq [1 x %"struct.ap_uint<128>"]* %src, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  call void @"arraycpy_hls.p0a1struct.ap_uint<128>"([1 x i128]* %dst, [1 x %"struct.ap_uint<128>"]* nonnull %src, i64 1)
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define void @"arraycpy_hls.p0a1struct.ap_uint<128>"([1 x i128]* nocapture "unpacked"="0.0" %dst, [1 x %"struct.ap_uint<128>"]* readonly "unpacked"="1" %src, i64 "unpacked"="2" %num) local_unnamed_addr #3 {
entry:
  %0 = icmp eq [1 x %"struct.ap_uint<128>"]* %src, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  %for.loop.cond1 = icmp sgt i64 %num, 0
  br i1 %for.loop.cond1, label %for.loop.lr.ph, label %copy.split

for.loop.lr.ph:                                   ; preds = %copy
  br label %for.loop

for.loop:                                         ; preds = %for.loop, %for.loop.lr.ph
  %for.loop.idx2 = phi i64 [ 0, %for.loop.lr.ph ], [ %for.loop.idx.next, %for.loop ]
  %src.addr.0.0.05 = getelementptr [1 x %"struct.ap_uint<128>"], [1 x %"struct.ap_uint<128>"]* %src, i64 0, i64 %for.loop.idx2, i32 0, i32 0, i32 0
  %dst.addr.0.0.06 = getelementptr [1 x i128], [1 x i128]* %dst, i64 0, i64 %for.loop.idx2
  %1 = load i128, i128* %src.addr.0.0.05, align 16
  store i128 %1, i128* %dst.addr.0.0.06, align 16
  %for.loop.idx.next = add nuw nsw i64 %for.loop.idx2, 1
  %exitcond = icmp ne i64 %for.loop.idx.next, %num
  br i1 %exitcond, label %for.loop, label %copy.split

copy.split:                                       ; preds = %for.loop, %copy
  br label %ret

ret:                                              ; preds = %copy.split, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @copy_out([11 x %"struct.ap_uint<512>"]* noalias "unpacked"="0", [11 x i512]* noalias nocapture readonly align 512 "unpacked"="1.0", [1 x %"struct.ap_uint<128>"]* noalias "unpacked"="2", [1 x i128]* noalias nocapture readonly align 512 "unpacked"="3.0") unnamed_addr #4 {
entry:
  call fastcc void @"onebyonecpy_hls.p0a11struct.ap_uint<512>"([11 x %"struct.ap_uint<512>"]* %0, [11 x i512]* align 512 %1)
  call fastcc void @"onebyonecpy_hls.p0a1struct.ap_uint<128>.1090"([1 x %"struct.ap_uint<128>"]* %2, [1 x i128]* align 512 %3)
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @"onebyonecpy_hls.p0a1struct.ap_uint<128>.1090"([1 x %"struct.ap_uint<128>"]* noalias "unpacked"="0" %dst, [1 x i128]* noalias nocapture readonly align 512 "unpacked"="1.0" %src) unnamed_addr #2 {
entry:
  %0 = icmp eq [1 x %"struct.ap_uint<128>"]* %dst, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  call void @"arraycpy_hls.p0a1struct.ap_uint<128>.1093"([1 x %"struct.ap_uint<128>"]* nonnull %dst, [1 x i128]* %src, i64 1)
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define void @"arraycpy_hls.p0a1struct.ap_uint<128>.1093"([1 x %"struct.ap_uint<128>"]* "unpacked"="0" %dst, [1 x i128]* nocapture readonly "unpacked"="1.0" %src, i64 "unpacked"="2" %num) local_unnamed_addr #3 {
entry:
  %0 = icmp eq [1 x %"struct.ap_uint<128>"]* %dst, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  %for.loop.cond1 = icmp sgt i64 %num, 0
  br i1 %for.loop.cond1, label %for.loop.lr.ph, label %copy.split

for.loop.lr.ph:                                   ; preds = %copy
  br label %for.loop

for.loop:                                         ; preds = %for.loop, %for.loop.lr.ph
  %for.loop.idx2 = phi i64 [ 0, %for.loop.lr.ph ], [ %for.loop.idx.next, %for.loop ]
  %src.addr.0.0.05 = getelementptr [1 x i128], [1 x i128]* %src, i64 0, i64 %for.loop.idx2
  %dst.addr.0.0.06 = getelementptr [1 x %"struct.ap_uint<128>"], [1 x %"struct.ap_uint<128>"]* %dst, i64 0, i64 %for.loop.idx2, i32 0, i32 0, i32 0
  %1 = load i128, i128* %src.addr.0.0.05, align 16
  store i128 %1, i128* %dst.addr.0.0.06, align 16
  %for.loop.idx.next = add nuw nsw i64 %for.loop.idx2, 1
  %exitcond = icmp ne i64 %for.loop.idx.next, %num
  br i1 %exitcond, label %for.loop, label %copy.split

copy.split:                                       ; preds = %for.loop, %copy
  br label %ret

ret:                                              ; preds = %copy.split, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @"onebyonecpy_hls.p0a11struct.ap_uint<512>.1100"([11 x i512]* noalias nocapture align 512 "unpacked"="0.0" %dst, [11 x %"struct.ap_uint<512>"]* noalias readonly "unpacked"="1" %src) unnamed_addr #2 {
entry:
  %0 = icmp eq [11 x %"struct.ap_uint<512>"]* %src, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  call void @"arraycpy_hls.p0a11struct.ap_uint<512>.1103"([11 x i512]* %dst, [11 x %"struct.ap_uint<512>"]* nonnull %src, i64 11)
  br label %ret

ret:                                              ; preds = %copy, %entry
  ret void
}

; Function Attrs: argmemonly noinline norecurse willreturn
define void @"arraycpy_hls.p0a11struct.ap_uint<512>.1103"([11 x i512]* nocapture "unpacked"="0.0" %dst, [11 x %"struct.ap_uint<512>"]* readonly "unpacked"="1" %src, i64 "unpacked"="2" %num) local_unnamed_addr #3 {
entry:
  %0 = icmp eq [11 x %"struct.ap_uint<512>"]* %src, null
  br i1 %0, label %ret, label %copy

copy:                                             ; preds = %entry
  %for.loop.cond1 = icmp sgt i64 %num, 0
  br i1 %for.loop.cond1, label %for.loop.lr.ph, label %copy.split

for.loop.lr.ph:                                   ; preds = %copy
  br label %for.loop

for.loop:                                         ; preds = %for.loop, %for.loop.lr.ph
  %for.loop.idx2 = phi i64 [ 0, %for.loop.lr.ph ], [ %for.loop.idx.next, %for.loop ]
  %src.addr.0.0.05 = getelementptr [11 x %"struct.ap_uint<512>"], [11 x %"struct.ap_uint<512>"]* %src, i64 0, i64 %for.loop.idx2, i32 0, i32 0, i32 0
  %dst.addr.0.0.06 = getelementptr [11 x i512], [11 x i512]* %dst, i64 0, i64 %for.loop.idx2
  %1 = load i512, i512* %src.addr.0.0.05, align 64
  store i512 %1, i512* %dst.addr.0.0.06, align 64
  %for.loop.idx.next = add nuw nsw i64 %for.loop.idx2, 1
  %exitcond = icmp ne i64 %for.loop.idx.next, %num
  br i1 %exitcond, label %for.loop, label %copy.split

copy.split:                                       ; preds = %for.loop, %copy
  br label %ret

ret:                                              ; preds = %copy.split, %entry
  ret void
}

declare i8* @malloc(i64)

declare void @free(i8*)

declare void @apatb_mpc_fpga_top_opencl_hw([11 x i512]*, i128*)

; Function Attrs: argmemonly noinline norecurse willreturn
define internal fastcc void @copy_back([11 x %"struct.ap_uint<512>"]* noalias "unpacked"="0", [11 x i512]* noalias nocapture readonly align 512 "unpacked"="1.0", [1 x %"struct.ap_uint<128>"]* noalias "unpacked"="2", [1 x i128]* noalias nocapture readonly align 512 "unpacked"="3.0") unnamed_addr #4 {
entry:
  call fastcc void @"onebyonecpy_hls.p0a1struct.ap_uint<128>.1090"([1 x %"struct.ap_uint<128>"]* %2, [1 x i128]* align 512 %3)
  ret void
}

declare void @mpc_fpga_top_opencl_hw_stub(%"struct.ap_uint<512>"* noalias nonnull readonly, %"struct.ap_uint<128>"* noalias nonnull)

define void @mpc_fpga_top_opencl_hw_stub_wrapper([11 x i512]*, i128*) #5 {
entry:
  %2 = call i8* @malloc(i64 704)
  %3 = bitcast i8* %2 to [11 x %"struct.ap_uint<512>"]*
  %4 = call i8* @malloc(i64 16)
  %5 = bitcast i8* %4 to [1 x %"struct.ap_uint<128>"]*
  %6 = bitcast i128* %1 to [1 x i128]*
  call void @copy_out([11 x %"struct.ap_uint<512>"]* %3, [11 x i512]* %0, [1 x %"struct.ap_uint<128>"]* %5, [1 x i128]* %6)
  %7 = bitcast [11 x %"struct.ap_uint<512>"]* %3 to %"struct.ap_uint<512>"*
  %8 = bitcast [1 x %"struct.ap_uint<128>"]* %5 to %"struct.ap_uint<128>"*
  call void @mpc_fpga_top_opencl_hw_stub(%"struct.ap_uint<512>"* %7, %"struct.ap_uint<128>"* %8)
  call void @copy_in([11 x %"struct.ap_uint<512>"]* %3, [11 x i512]* %0, [1 x %"struct.ap_uint<128>"]* %5, [1 x i128]* %6)
  call void @free(i8* %2)
  call void @free(i8* %4)
  ret void
}

attributes #0 = { noinline willreturn "fpga.wrapper.func"="wrapper" }
attributes #1 = { argmemonly noinline norecurse willreturn "fpga.wrapper.func"="copyin" }
attributes #2 = { argmemonly noinline norecurse willreturn "fpga.wrapper.func"="onebyonecpy_hls" }
attributes #3 = { argmemonly noinline norecurse willreturn "fpga.wrapper.func"="arraycpy_hls" }
attributes #4 = { argmemonly noinline norecurse willreturn "fpga.wrapper.func"="copyout" }
attributes #5 = { "fpga.wrapper.func"="stub" }

!llvm.dbg.cu = !{}
!llvm.ident = !{!0, !0, !0, !0, !0, !0, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1, !1}
!llvm.module.flags = !{!2, !3, !4}
!blackbox_cfg = !{!5}

!0 = !{!"AMD/Xilinx clang version 16.0.6"}
!1 = !{!"clang version 7.0.0 "}
!2 = !{i32 2, !"Dwarf Version", i32 4}
!3 = !{i32 2, !"Debug Info Version", i32 3}
!4 = !{i32 1, !"wchar_size", i32 4}
!5 = !{}
