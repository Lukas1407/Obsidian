- Because of the mentioned problems of using [[Intersection Over Union (IoU)]], Average Precision (AP) is generally used as an evaluation index for instance segmentation 
- The AP metric computes the area under the Precision-Recall curve. With Precision and Recall being defined as followed: $$Precision = \frac{TP}{TP+FP},$$$$Recall = \frac{TP}{TP+FN} $$
- Here True Positives (TP) are the masks that were correctly predicted, meaning the IoU between the mask and the ground truth (GT) is greater than some threshold $t$. 
- False Positives (FP) are the masks whose IoU is not greater than the threshold, meaning an instance was predicted where there is none. 
- False Negatives (FN) are the masks where the model failed to identify an instance that was present. 
- To classify a mask as TP or FP, the [[Intersection Over Union (IoU)|IoU]] between the predicted mask and its GT mask is calculated. 
	- For example AP50 refers to the threshold being 50%. 
	- 
- Precision measures the percentage of correct positive predictions among all predictions made, so it reflects how precise the model is. 
	- However, precision alone is not a reliable metric, because it can be inflated by making fewer predictions. For example, if one prediction is made and it is correct, the precision is 1, regardless of how many actual positive cases are missed. 
- Recall measures the percentage of correct positive predictions among all actual positive cases, so it reflects how well the model can identify all the relevant cases. 
	- However, recall alone is also not a reliable metric, because it can be inflated by making more predictions. For example, if all pixels are predicted as positive, the recall is 1, regardless of how many FP are generated. 
- This is why AP measures a combination of both, the area under the Precision-Recall curve 
- The precision recall curve is a graphical representation that shows how the model performs in terms of both precision and recall at different levels of confidence. The confidence level is a threshold that determines whether a prediction is considered positive or negative. A higher confidence level means that the model is more selective in making predictions, but it may also miss some TP. A lower confidence level means that the model is more inclusive in making predictions, but it may also generate more FP. The precision recall curve plots the precision and recall values for each confidence level. The area under the curve (AUC) is a metric that summarizes the overall quality of the model. A higher AUC value means that the model can achieve high precision and high recall simultaneously, which is desirable for instance segmentation. Measuring the precision and recall at a single IoU threshold comes with a few problems. A single low IoU threshold might categorize a detection as TP, even if it only slightly overlaps with the GT. Conversely, using a single high IoU threshold could label a detection as a FP, despite it encompassing the majority of the GT. This can lead to overestimating or underestimating the model’s performance. 53 8 Evaluation Methods Because of this dependence on the IoU threshold, mean Average Precision (mAP) is used to average over multiple IoU thresholds as well as over all classes [90]. For evaluating the mAP on the COCO dataset (section 7.1) IoU thresholds from 0.5 to 0.95 with a step size of 0.05 are averaged. In this thesis, the term AP denotes the mean Average Precision (mAP), which is commonly done in literature.