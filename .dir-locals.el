;; .dir-locals.el for basic c++ project set up

((nil
  (helm-make-build-dir . "planlab_ASP/build"))
 (c++-mode
  (helm-make-arguments . "-j8")
  (flycheck-gcc-language-standard . "c++11")))
