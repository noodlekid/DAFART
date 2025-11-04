% simple prop

tx = txsite;
pm = propagationModel("longley-rice","TimeVariabilityTolerance",0.7);

coverage(tx,"PropagationModel",pm);