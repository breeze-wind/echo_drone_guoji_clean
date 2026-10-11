"""CUDA-tensor fallback for TorchVision builds missing the CUDA NMS kernel.

The detector and all NMS tensor math remain on the input device. This exists
for Jetson wheels where torchvision::nms was compiled with a CPU kernel only.
"""


def _torch_nms(boxes, scores, iou_threshold):
    """Greedy NMS implemented with torch ops on boxes.device (including CUDA)."""
    import torch

    if boxes.numel() == 0:
        return torch.empty((0,), dtype=torch.long, device=boxes.device)

    # IoU arithmetic in fp32 is more stable while remaining on the GPU.
    boxes = boxes.to(dtype=torch.float32)
    scores = scores.to(device=boxes.device)
    order = torch.argsort(scores, descending=True)
    kept = []

    while order.numel() > 0:
        current = order[0]
        kept.append(current)
        if order.numel() == 1:
            break

        rest = order[1:]
        x1 = torch.maximum(boxes[current, 0], boxes[rest, 0])
        y1 = torch.maximum(boxes[current, 1], boxes[rest, 1])
        x2 = torch.minimum(boxes[current, 2], boxes[rest, 2])
        y2 = torch.minimum(boxes[current, 3], boxes[rest, 3])
        intersection = (x2 - x1).clamp(min=0) * (y2 - y1).clamp(min=0)
        area_current = ((boxes[current, 2] - boxes[current, 0]).clamp(min=0) *
                        (boxes[current, 3] - boxes[current, 1]).clamp(min=0))
        area_rest = ((boxes[rest, 2] - boxes[rest, 0]).clamp(min=0) *
                     (boxes[rest, 3] - boxes[rest, 1]).clamp(min=0))
        union = area_current + area_rest - intersection
        iou = intersection / union.clamp(min=torch.finfo(torch.float32).eps)
        order = rest[iou <= iou_threshold]

    return torch.stack(kept).to(dtype=torch.long)


def install_cuda_nms_fallback(logger=None):
    """Patch torchvision.ops.nms only when the installed CUDA kernel is absent."""
    import torchvision

    native_nms = torchvision.ops.nms
    native_cuda_status = None

    def compatible_nms(boxes, scores, iou_threshold):
        nonlocal native_cuda_status
        if not boxes.is_cuda or native_cuda_status is False:
            if boxes.is_cuda:
                return _torch_nms(boxes, scores, iou_threshold)
            return native_nms(boxes, scores, iou_threshold)

        try:
            result = native_nms(boxes, scores, iou_threshold)
            native_cuda_status = True
            return result
        except RuntimeError as exc:
            message = str(exc)
            missing_cuda_kernel = (
                'torchvision::nms' in message and
                ('CUDA' in message or 'cuda' in message) and
                ('not available' in message or 'Could not run' in message))
            if not missing_cuda_kernel:
                raise
            native_cuda_status = False
            if logger is not None:
                logger.warning(
                    'TorchVision CUDA NMS kernel is missing; using GPU-tensor '
                    'NMS fallback (inference and NMS remain on CUDA).')
            return _torch_nms(boxes, scores, iou_threshold)

    torchvision.ops.nms = compatible_nms
    return _torch_nms
