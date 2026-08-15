class dbf_agent extends uvm_agent;

  `uvm_component_utils(dbf_agent)

  dbf_agent_config cfg;
  dbf_sequencer    sequencer;
  dbf_driver       driver;
  dbf_monitor      monitor;

  function new(string name = "dbf_agent", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    if (!uvm_config_db#(dbf_agent_config)::get(this, "", "cfg", cfg)) begin
      `uvm_fatal(get_type_name(), "dbf_agent_config was not configured")
    end

    uvm_config_db#(
      virtual dbf_cal_chl_if #(DBF_DATA_WIDTH, DBF_ADDR_WIDTH)
    )::set(this, "monitor", "vif", cfg.vif);
    monitor = dbf_monitor::type_id::create("monitor", this);

    if (cfg.is_active == UVM_ACTIVE) begin
      uvm_config_db#(
        virtual dbf_cal_chl_if #(DBF_DATA_WIDTH, DBF_ADDR_WIDTH)
      )::set(this, "driver", "vif", cfg.vif);
      sequencer = dbf_sequencer::type_id::create("sequencer", this);
      sequencer.vif = cfg.vif;
      driver    = dbf_driver::type_id::create("driver", this);
    end
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);
    if (cfg.is_active == UVM_ACTIVE) begin
      driver.seq_item_port.connect(sequencer.seq_item_export);
    end
  endfunction

endclass
