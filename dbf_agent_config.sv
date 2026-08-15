class dbf_agent_config extends uvm_object;

  `uvm_object_utils(dbf_agent_config)

  virtual dbf_cal_chl_if #(DBF_DATA_WIDTH, DBF_ADDR_WIDTH) vif;
  uvm_active_passive_enum is_active = UVM_ACTIVE;

  function new(string name = "dbf_agent_config");
    super.new(name);
  endfunction

endclass
