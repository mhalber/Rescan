# Evaluates semantic label task
# Input:
#   - path to .txt prediction files
#   - path to .txt ground truth files
#   - output file to write results to
# Note that only the valid classes are used for evaluation,
# i.e., any ground truth label not in the valid label set
# is ignored in the evaluation.
#
# example usage: evaluate_semantic_label.py --scan_path [path to scan data] --output_file [output file]

# python imports
import math
import os, sys, argparse
import inspect

try:
    import numpy as np
except:
    print("Failed to import numpy package.")
    sys.exit(-1)
try:
    from itertools import izip
except ImportError:
    izip = zip

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)
import util
import util_3d

parser = argparse.ArgumentParser()
parser.add_argument('--pred_path', required=True, help='path to directory of predicted .txt files')
parser.add_argument('--gt_path', required=True, help='path to gt files')
parser.add_argument('--output_file', default=None, help='optional output file')
opt = parser.parse_args()

CLASS_LABELS = ['unlabelled', 'wall','floor','cabinet','bed','chair','sofa','table','door','window','bookshelf','picture','counter','blinds','desk','shelves','curtain','dresser','pillow','mirror','floor_mat','clothes','ceiling','books','refridgerator','television','paper','towel','shower_curtain','box','whiteboard','person','night_stand','toilet','sink','lamp','bathtub','bag','otherstructure','otherfurniture','otherprop']
VALID_CLASS_ID = np.array([ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40 ])
VALID_INSTANCE_ID = np.array(range(0, 256))
UNKNOWN_ID = np.max(VALID_INSTANCE_ID) + 1


def evaluate_scan(pred_file, gt_file, confusion):
    try:
        pred_ids = util_3d.load_ids(pred_file)
    except Exception as e:
        util.print_error('unable to load ' + pred_file + ': ' + str(e))
    try:
        gt_ids = util_3d.load_ids(gt_file)
    except Exception as e:
        util.print_error('unable to load ' + gt_file + ': ' + str(e))

    # sanity checks
    pred_ids = pred_ids[0:gt_ids.shape[0]]
    if not pred_ids.shape == gt_ids.shape:
        util.print_error('%s: number of predicted values does not match number of vertices' % pred_file, user_fault=True)
    for (gt_val,pred_val) in izip(gt_ids.flatten(),pred_ids.flatten()):
        gt_instance_val = gt_val % 1000
        pred_instance_val = pred_val % 1000
        if gt_instance_val not in VALID_INSTANCE_ID:
            continue
        if pred_instance_val not in VALID_INSTANCE_ID:
            pred_val = UNKNOWN_ID
        confusion[gt_instance_val][pred_instance_val] += 1
    return gt_ids

def reorder_confusion( confusion, eq_file ):
    equivalences = eq_file.readlines()
    for eq in equivalences:
        # print( eq.rstrip() )
        eq_vals = eq.rstrip().replace('|', ' ').split()
        base_id = int(eq_vals[0]) + 1
        eq_ids  = [int(x) + 1 for x in eq_vals[1:]]
        # print( base_id, eq_ids )
        vals = [ confusion[base_id][eq_id] for eq_id in eq_ids ]
        best_id = eq_ids[vals.index(max(vals))]
        if base_id != best_id:
            confusion[base_id][base_id], confusion[base_id][best_id] = confusion[base_id][best_id], confusion[base_id][base_id]


def get_iou(label_id, confusion):
    if not label_id in VALID_INSTANCE_ID:
        return (float('nan'), 0, 0 )
    # #true positives
    tp = np.longlong(confusion[label_id, label_id])
    # #false negatives
    fn = np.longlong(confusion[label_id, :].sum()) - tp
    # #false positives
    not_ignored = [l for l in VALID_INSTANCE_ID if not l == label_id]
    fp = np.longlong(confusion[not_ignored, label_id].sum())

    denom = (tp + fp + fn)
    if denom == 0:
        return (float('nan'), 0, 0 )
    return (float(tp) / denom, tp, denom)


def evaluate(pred_files, gt_files, output_filename):
    max_id = UNKNOWN_ID

    print('evaluating', len(pred_files), 'scans...')
    mean_mean_iou = 0
    output_file = None
    if output_filename:
        print("Writing to ", output_filename)
        output_file = open(output_filename, 'w')

    for i in range(len(pred_files)):
        subsequence_name = os.path.basename(pred_files[i])
        sequence_name = subsequence_name[:subsequence_name.rfind('_')]
        eq_file_path = os.path.join( sequence_name, 'gt_segmentation', subsequence_name)
        eq_file = None
        if os.path.exists( eq_file_path ):
            eq_file = open( eq_file_path, 'r')
        confusion = np.zeros((max_id+1, max_id+1), dtype=np.ulonglong)
        gt_ids = evaluate_scan(pred_files[i], gt_files[i], confusion)
        if eq_file:
            reorder_confusion( confusion, eq_file )
            eq_file.close()
        valid_ids = np.sort(np.unique(gt_ids))
        if valid_ids[0] == 0:
            valid_ids = valid_ids[1:]
        valid_instance_ids = [ int(x % 1000) for x in valid_ids ]
        valid_semantic_ids = [ int(x // 1000) for x in valid_ids ]
        valid_ids = sorted(zip(valid_instance_ids, valid_semantic_ids))

        mean_iou = 0
        if output_file:
            output_file.write( "Scan #%-3d: %s\n" % (i+1, subsequence_name))
            output_file.write( "%20s %9s %8s %8s\n" %("Instance name", "IOU", "TP", "Total" ))
        for val in valid_ids:
            gt_instance_val = val[0]
            gt_semantic_val = val[1]
            iou = get_iou( gt_instance_val, confusion)
            mean_iou += iou[0]
            instance_name = CLASS_LABELS[gt_semantic_val] + '_' + str(gt_instance_val)
            if output_file:
                result_str = "%20s %8.7f %8d %8d\n" %( instance_name, iou[0], iou[1], iou[2])
                output_file.write( result_str )

        mean_iou /= len(valid_ids)
        mean_mean_iou += mean_iou
    
        if output_file:
            output_file.write("Mean IOU %8.7f\n\n" % (mean_iou))
    
        print("Processed scan #%-3d (%s) -> Mean IOU %8.7f" % (i+1, subsequence_name, mean_iou))
    
    mean_mean_iou /= len(pred_files)
    print('\nAverage Mean IOU: %8.7f' % (mean_mean_iou))

    if output_file:
        output_file.write('Average Mean IOU: %8.7f\n' % (mean_mean_iou))
        output_file.close()

def main():
    pred_files = [f for f in os.listdir(opt.pred_path) if f.endswith('.txt') and f != 'instance_transfer_evaluation.txt']
    gt_files = []
    if len(pred_files) == 0:
        util.print_error('No result files found.', user_fault=True)
    for i in range(len(pred_files)):
        gt_file = os.path.join(opt.gt_path, pred_files[i])
        if not os.path.isfile(gt_file):
            util.print_error('Result file {} does not match any gt file'.format(pred_files[i]), user_fault=True)
        gt_files.append(gt_file)
        pred_files[i] = os.path.join(opt.pred_path, pred_files[i])

    evaluate(pred_files, gt_files, opt.output_file)


if __name__ == '__main__':
    main()
